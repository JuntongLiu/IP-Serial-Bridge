/*
 *   
 *   Juntong Liu
 *   @European Spallation Source(ESS)
 *   email: juntong.liu@ess.eu
 *                                       2020.12.02
 *   ======================================================
 * 
 *   This is a simple TCP/IP--Serial bridge server which bridges a TCP/IP network and a serial network. It is designed for 
 *   enabling the communication between IOC and LakeShore240 temperature sensor input module. 
 *   This program was originally designed for ESS, but not used anymore.
 *   
 *   The server creates two threads to serve two clients: one is IOC's monitor plane and one is IOC's control plane.
 *   Communications are initialled by the two clients in the IOC. When this server receive an IP packet from the IOC, it 
 *   constructs a serial frame and send it to LakeShore-240 over the virtual serial port utilizing the physical USB connection,
 *   and then read the reply from LakeShore240. After received the LakeShore-240's reply, it sends the reply over the TCP/IP 
 *   network back to the client (IOC's control plane or monitor plane).
 *
 *   Here, due to the special case that it is not possible to do multiplexing and de-multiplexing over the serial
 *   connection, every time this server send a request to LakeShore-240, it must wait for LakeShore240's reply. Otherwise, 
 *   other thread may jump in and mess up the communication. This means that a round of querry-and-reply over the serial link 
 *   must be finished without being interrupted by other thread. This is achieved by employing mutex to guarantee that each 
 *   thread can exclusively use the serial link to communicate with LakeShore-240 until a round of query-reply completed. 
 *
 *   This server listen on TCP port 5002.  
 *   And the TCP clients in the IOC must be set up to connect this server using port 5002.
 *
 *   
 *  /Library/Developer/CommandLineTools/SDKs/MacOSX10.14.sdk/usr/include/sys/socket.h
 * 
 *  On Mac or Linux, do:    
 *                          gcc -Wall IPSerialBridge.c -o IPSerialBridge
 * 
 *  On Raspberry, do:  
 *                          gcc -Wall -pthread IPSerialBridge.c -o IPSerialBridge
 * 
 *  And to start the server, type:
 *                                ./IPSerialBridge
 * 
 *  Primary tests show that this server works fine with the IOC for LakeShore-240!
 * 
 * 
 *  Please Note, this program is working, but it is still in a primary state, many thing need to be added to make it a real 
 *  complete program. To use it is at your own risk.
 */  

#include <stdio.h> 
#include <netdb.h> 
#include <netinet/in.h> 
#include <stdlib.h> 
#include <string.h> 
#include <sys/socket.h> 
#include <sys/types.h> 
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>  
#include <termios.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/signal.h>
//#include <linux/kernel.h>
#include <stdarg.h>
#include <time.h>  // for nanosleep()

#define NUM_THREADS 2

#define MAX 80 
//#define PORT 5002   // 40000   // 55555 8080 

#define SA struct sockaddr 
#define PORT    5002    //5555
#define MAXMSG  512
#define BUFFLEN 512
char rcv_buff[BUFFLEN];

struct termios savetty;

int com_fd; 
pthread_mutex_t mutexSerialFd_lock;      // Use a mutex to synchronize the use of the serial fd and connection

#define DEBUG
#undef DEBUG
#ifdef DEBUG
#define dprintf(format, args...) printf(format, ##args)
#define DBGPRINT(fmt, ...)                             \
   do                                                  \
   {                                                   \
         fprintf(stderr, "%s:%d:%s(): " fmt, __FILE__, \
                 __LINE__, __func__, __VA_ARGS__);     \
   } while (0)
#else
#define dprintf(format, args...)
#define DBGPRINT(fmt, ...)
#endif

#if 0
#ifdef DEBUG
#define DEBUG_print(fmt, ...)                          \
   do                                                  \
   {                                                   \
      if (DEBUG)                                       \
         fprintf(stderr, "%s:%d:%s(): " fmt, __FILE__, \
                 __LINE__, __func__, __VA_ARGS__);     \
   } while (0)
#else
#define DEBUG_print(fmt, ...)
#endif 
#endif 

/**-----------------------------------------------------------------------------------------------------------------------------------
 * @brief  Delay 10 million second. Lakeshore240 requires a delay of 10 million seconds between commands. 
 * @note   
 * @retval 0 = delay 10ms; positive value mean under-delayed; negtive value mean over-delayed
 */
static int wait_10ms(void)
{
   //int ret;
   struct timespec tim, tim2;
   tim.tv_sec = 0;
   tim.tv_nsec = 1e+7;           // 10ms  milliom second, 1000,000,000 nanoseconds = 1 second
   return nanosleep(&tim, &tim2);
 
}

/* This program is going to run on two platforms, one is Raspberry Pi and another one is Mac OSX. The initialization of the serial port on 
* these two platforms are a little different, so, here we have two functions, one for each platform. */

#undef RASPBERRYPI
#ifdef RASPBERRYPI
/**-----------------------------------------------------------------------------------------------------------------------------------
 * @brief  The init_serial_ports() function initialize the serial over USB virtual serial port. This function is for Raspberrypi.
 * @note   
 * @retval 
 */
static int init_serial_ports(void)             // for Raspberry Pi
{
   struct termios tty;
   speed_t spd;
   int rc;

#ifdef RASPBERRYPI
   com_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
#else
   com_fd = open("/dev/tty.SLAB_USBtoUART", O_RDWR | O_NOCTTY);
#endif 

   if(com_fd < 0)
   {
      perror("Error open virtual serial port!");
      return -1;
   } 
   rc = tcgetattr(com_fd, &tty);
   if(rc < 0)
   {
      perror("Error get old tty parameters!");
      return -2;
   }
   savetty = tty;

   spd = B115200;
   cfsetospeed(&tty, (speed_t)spd);
   cfsetispeed(&tty, (speed_t)spd);

   cfmakeraw(&tty);

   tty.c_cc[VTIME] = 2; //10;             // set terminal i/o time out, 2 ok, 5 do not work
   tty.c_cc[VMIN] = 0;  //1;              // blocking read util minimum 5 char received

   tty.c_cflag &= ~CSTOPB;
   tty.c_cflag &= ~CRTSCTS;   // NO hw flow control 
   tty.c_cflag |= CLOCAL | CREAD;
   rc = tcsetattr(com_fd, TCSANOW, &tty);
   if(rc < 0){
      perror("ERROR set new tty attributs!");
      return -3;
   }

    printf("Serial port initialization OK!\n");

    return 0;

}

#else                     // for MAC laptop 

/**-----------------------------------------------------------------------------------------------------------------------------------
 * @brief  Called by main to initialize the virtual port - set the baud rate, ...etc.
 * @note   
 * @retval 0 -- OK,   -1 -- error 
 */
static int init_serial_ports(void)
{
   int val;
   struct termios newOpt, oldOpt; // options;
   // Open the virtual port
   //com_fd = open(serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
   com_fd = open("/dev/tty.SLAB_USBtoUART", O_RDWR | O_NOCTTY | O_NONBLOCK);
   //com_fd = open(serial_port, O_RDWR | O_NOCTTY);
   if (com_fd < 0)
   {
      printf("################################\n");
      printf("Error opening USB virtual port!!!\n");
      printf("################################\n");
      return -1;
   };

   val = fcntl(com_fd, F_GETFL, 0);
   printf("DEBUG::: 1. serail port opened, file status = 0x%x\n", val);

   bzero(&newOpt, sizeof(struct termios));
   /* set terminal attributes.  */
   //  fcntl(com_fd[i], F_SETFL, 0);
   tcgetattr(com_fd, &oldOpt);
   cfsetispeed(&newOpt, 115200);
   cfsetospeed(&newOpt, 115200);

   /* xia mian xian tong, following is the first one work on com6 */
#if 0    
   newOpt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
			       | INLCR | IGNCR | ICRNL | IXON);
   newOpt.c_oflag &= ~OPOST;
   newOpt.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
   newOpt.c_cflag &= ~(CSIZE | PARENB);
   newOpt.c_cflag |= CS8; 
#endif
   // newOpt.c_lflag |= (ICANON | ECHO | ECHOE);
   // newOpt.c_iflag |= (IGNPAR); // | IXON | IXOFF | IXANY);
   // newOpt.c_iflag |= (IXON | IXOFF | IXANY);     // enable software flow control
   // newOpt.c_oflag |= OPOST;
#if 1   // ----- active one    
   newOpt.c_cflag |= (CS8 | CLOCAL | CREAD);
   newOpt.c_iflag &= ~(IXON | IXOFF | IXANY);      // disable software flow control
   newOpt.c_oflag = 0;                             // oflag=0, use raw output
   newOpt.c_cc[VTIME] = 2;                         // set terminal i/o time out, 2 ok, 5 do not work
   newOpt.c_cc[VMIN] = 0;                          // blocking read util minimum 5 char received
   dprintf("DEBUG::: CS8, ... parameter set done\n");
#endif

#if 0 // -------------------
// Set close on exec flag
#if defined(FD_CLOEXEC)
   if (fcntl(com_fd, F_SETFD, FD_CLOEXEC) < 0)
   {
      dprintf( "Can't set close-on-exec flag!\n");
      close(com_fd);
      com_fd = -1;
      return -1;
   }
   printf("DEBUG::: fctl() called  f_setfd, FD_CLOEXEC... done\n");
#endif 
#endif 

      
   // Clean the line and active new setting
   tcflush(com_fd, TCIFLUSH);

   int tcret = tcsetattr(com_fd, TCSANOW, &newOpt); 
   if(tcret < 0)
   {
      printf("tcsetattr() failed!\n");
	   close(com_fd);
      com_fd = -1;
      return -1;
   }
   //dprintf("com_fd = %d\n; ", com_fd);

   tcflush(com_fd, TCIOFLUSH);
   //tty->readTimeout = -1e-99;
   //tty->writeTimeout = -1e-99;
#if 1 // <<<--------------- Set this cause the USB port blocked on raspberry Pi 4!!!!!
   if (fcntl(com_fd, F_SETFL, 0) < 0)
   {
      dprintf("Can't set F_SETFL file flags!\n");
      close(com_fd);
      com_fd = -1;
      return -1;
   }
   val = fcntl(com_fd, F_GETFL, 0);
   printf("DEBUG::: 2. serail port opened, file status now = 0x%x\n", val);
   printf("DEBUG::: fctl set F_SETFL, ...  done\n");
   //dprintf("\n Port fd=%d opened and options set\n", com_fd);
#endif 

#if 0   
   /* get the old settings */
   tcgetattr(com_fds[0], &old_settings);

   /* prepare to set the attributes */
   new_settings.c_cflag = BAUD | CRTSCTS | DATABITS | STOPBITS | PARITYON | PARITY | CLOCAL | CREAD;
   new_settings.c_iflag = IGNPAR;
   new_settings.c_oflag = 0;
   new_settings.c_lflag = 0;          //ICANON;
   new_settings.c_cc[VMIN]=1;
   newsettings.c_cc[VTIME]=0;

   /* set ports' new attributes */
   for (i=1; i<3; i++)
   {
      tcflush(com_fds[i], TCIFLUSH);
      if(tcsetattr(fd, TCSANOW, &newtio) < 0)
      {
         sprintf(term_out, "%s\n", "################################");
         sprintf(term_out, "Serial port %d initialization failed!\n", i);
         sprintf(term_out, "%s\n", "################################");
         exit(-1);
      }
   }
#endif 
     
   printf("Serial port initialization done, com_fd=%d\n", com_fd);
   return 0; 
}

#endif  // ifdef RASPBERRYPI

/**--------------------------------------------------------------------------------------------------------------------------------
 * @brief  Called after a query command is sent to Lakeshore240 to get the device's reply
 * @note   
 * @retval 
 */
static int ls240_getReply(void)
{
   ssize_t rw_ret;
   char buff[8];
   int tries = 0, ret;
   ret = wait_10ms();
   DBGPRINT("nanosleep() returned with: %d\n", ret);
   // When we run this server on another machine, like a Raspberrypi, 
   //while (1)   // We have blocking-reading now 
   {
      //pos++;
      tries++;
      rw_ret = read(com_fd, rcv_buff, BUFFLEN - 1);             /* read the reply */

      // if (strstr(rcv_buff, "\r\n") || strchr(rcv_buff, '\n'))   // we have block reading now
      //    break;
      
      if (rw_ret < 0)
      {
         int val;
         perror("Read error from 240!");
         
         // --------- For DEBUG ONLY -----------
         //val = fcntl(com_fd, F_GETFL, 0);
         //printf("DEBUG::: File status = 0x%x\n", val);
         //-------------------------------------
         
         // To increase the stability, do not close the fd when one read/write error happen. Instead, do a TC FLUSH to clean 
         // the possible problem caused by the faild read/write, which may has bad effect on the following read/write operation.
         // And let the following read/write continue.
         //close(com_fd);
         //com_fd = -1;
         tcflush(com_fd, TCIOFLUSH);
         return -1;
      }
   }

   // For a kind of flow control
   //snprintf(buff, 8, "%c", '\n');
   //rw_ret = write(com_fd, buff, strlen(buff));
   //if (rw_ret < 0)
   //{
   //   perror("Error write to 240!\n");
   //   close(com_fd);
   //   return -1;
   //}
   // We do a flush instead
   tcflush(com_fd, TCIOFLUSH);

   dprintf("Get ls240 reply after %d read() tries: \'%s\'\n", tries, rcv_buff);
   dprintf("IPSerialBridge <<========= LakeShore240: \'%s\'\n", rcv_buff);
   return 0;
}

/**-----------------------------------------------------------------------------------------------------------------------------------
 * @brief  This is the main thread function to perform bridging between TCP/IP and Serial network.
 * @note   
 * @param  *connfd: This is the sock fd, from which, the TCP packet including the query from the IOC is received.
 * @retval 
 */
void * thread_func(void *connfd)
{ 
   char buff[MAX]; 
   int n; 
   //long tid;
   //tid = (long)threadid;
   int sockfd = (int) connfd;
   dprintf("DEBUG::: in thread_func(), sockfd=%d\n", sockfd);
    
   // infinite loop for process LakeShore-240 queries and replies 
   for (;;) 
   { 
      bzero(buff, MAX); 
      bzero(rcv_buff, 256);
        
      // 1.) IP to Serial bridging
        
      read(sockfd, buff, sizeof(buff));   // read the TCP socket to get the query from the IOC 

      //if(strlen(buff) > 1)    // lakeshore-240 flow control flush make this dose not work on Raspberry, change it to 2
      if(strlen(buff) > 2)      // This will make the Raspberry Pi happy
      { 
         dprintf("(IOC ====>> IPSerialBridge), Contents(fd=%d), msglen=%lu: \'%s\'\n ", sockfd,  strlen(buff), buff);

         /* Send the IOC's querry to LakeShore-240 over the serial-over-USB virtual port. But, we need to
          * get the mutex before we write the query from IOC to LakeShore-240, so that another thread can not interrupt our
          * communication with the LakeShore240 over the serial link  */
         pthread_mutex_lock(&mutexSerialFd_lock);

         // we flush the line before write a new query
         tcflush(com_fd, TCIOFLUSH);

         write(com_fd, buff, sizeof(buff));
         dprintf("(IPSerialBridge ====>> LakeShore240). Contents(fd=%d): \'%s\'\n", sockfd, buff);

         // wait 10ms 
         wait_10ms();

         // get the LS240 reply
         if(ls240_getReply() < 0)
         {
            printf("Error reading ls240s reply!\n");
            // we continue if only one reading is not right;
         }
         dprintf("(IPSerialBridge <<==== LakeShore240). Contents(fd=%d): \'%s\'\n", sockfd, rcv_buff);
         
         //bzero(buff, MAX); 
         strncpy(buff, rcv_buff, sizeof(buff));
         n = 0;

         // a round of query-reply is done over the serial link, free the mutex so another thread can access the serial link
         pthread_mutex_unlock(&mutexSerialFd_lock);
      
      }
      else
      {
         // The IOC sends one byte frame as flow control package, so we do a flush, instead of sending the empty package to LakeShore-240
         tcflush(com_fd, TCIOFLUSH);
      }

      // 2.) Serial to IP bridging

      // Now, a round of query-reply is completed from LakeShore240's point of view, we get the LakeShore-240's reply. 
      // Now send it to the IOC 
      //if(strlen(buff) > 1)
      if(strlen(buff) > 2)
      {
         // Send the LakeShore-240's reply to the plane of the IOC identified by the sockfd
         write(sockfd, buff, sizeof(buff));    
         dprintf("(IOC <<==== IPSerialBridge). Contents(fd=%d): \'%s\'\n\n", sockfd, buff);
      }
   
   } 
   pthread_exit(NULL);

} 
  

/**--------------------------------------------------------------------------------------------------------------------------------
 * @brief  The main function of the TCP/IP---Serial bridge server
 * @note   
 * @retval 
 */
int main() 
{ 
   long sockfd, connfd[2];
   socklen_t len; 
   int pthread_ret[2]; 
   pthread_t threads[NUM_THREADS];
   struct sockaddr_in servaddr, cli; 
  
   struct timeval t;
   t.tv_sec = 0;
   t.tv_usec = 0;
   printf("\n#######################################################\n");
   printf("   TCP/IP <--> Serial bridge server is starting......\n");
   printf("#######################################################\n\n");
   // prepare the thread to use mutex, so that it can exclusively use the serial link in a round of query-reply
   pthread_attr_t attr;
   pthread_mutex_init(&mutexSerialFd_lock, NULL);
   pthread_attr_init(&attr);
   pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

   // initial the serial port
   if(init_serial_ports() < 0)
   {
      printf("Error! initialize serial port failed!\n");
      exit(-1);
   }

   // TCP socket create and verification 
   sockfd = socket(AF_INET, SOCK_STREAM, 0); 
   if (sockfd == -1) 
   { 
      perror("Create socket");
      printf("socket creation failed...\n"); 
      exit(0); 
   } 
   else
      printf("\nTCP Socket successfully created...\n"); 
   
   bzero(&servaddr, sizeof(servaddr)); 
  
   // assign IP address, PORT 
   servaddr.sin_family = AF_INET; 
   servaddr.sin_addr.s_addr = htonl(INADDR_ANY); 
   servaddr.sin_port = htons(PORT); 
  
   // Binding newly created socket to given IP and verification 
   if ((bind(sockfd, (SA*)&servaddr, sizeof(servaddr))) != 0)
   { 
      printf("socket bind failed...\n"); 
      exit(0); 
   } 
   else
      printf("TCP Socket successfully binded...\n"); 
  
   /* We know that there are 2 TCP clients in the IOC will try to set up connection with us. One is the control plane and the other is
    * mornitor plane. So, here we waiting for their connection request and create a thread to serve each plane */
   for(int i=0; i<2; i++)
   {
      // Now server is ready to listen and verification clients connection requests.
      if ((listen(sockfd, 5)) != 0)
      { 
         printf("Listen failed...\n"); 
         exit(0); 
      }    
      else
         printf("\nTCP Server listening......\n"); 
        
      len = sizeof(cli); 
  
      // Accept the connection request from client and verification 
      connfd[i] = accept(sockfd, (SA*)&cli, &len); 
      if (connfd[i] < 0) 
      { 
         printf("server accept failed...\n"); 
         exit(0); 
      } 
      else
         printf("TCP server accept a client's connection request, sock fd=%ld\n", connfd[i]); 
  
      dprintf("DEBUG::: Client accepted, sockfd = %ld\n", connfd[i]);
        
      // set the socket option and put the socket in a block-reading mode
      if(setsockopt(connfd[i], 
                    SOL_SOCKET,         // To manipulate options at socket API level 
                    SO_RCVTIMEO,        // Specify the receiveing or sending time out
                    (const void *)(&t), // option value 
                    sizeof(t)) != 0)
      {
         perror("setsocketopt()");
         printf("Error set socket option\n");
         exit(-1);
      };

      // Create a thread to serve the connection
      //pthread_ret[i] = pthread_create(&threads[i], NULL, thread_func, (void *)connfd[i]);
      pthread_ret[i] = pthread_create(&threads[i], &attr, thread_func, (void *)connfd[i]);
      printf("pthread_create() returned: %d, a thread has been created for sock fd=%ld\n", pthread_ret[i], connfd[i]);
      if(pthread_ret[i])
      {
         printf("ERROR, return from pthread_create() is %d\n", pthread_ret[i]);
         exit(-1);
      }   
  
   }

    pthread_join(threads[0], NULL);
    pthread_join(threads[1], NULL);
   
   
   pthread_attr_destroy(&attr);

   // After done close the socket 
   for(int i = 0; i < 2; i++ )
      close(connfd[i]); 

   return 0;
} 

