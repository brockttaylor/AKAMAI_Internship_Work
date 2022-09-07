/* -*- c-file-style: "Ellemtel" -*- */
/* Copyright (C) 2022   Canada-France-Hawaii Telescope Corp.          */
/* This program is distributed WITHOUT any warranty, and is under the */
/* terms of the GNU General Public License, see the file COPYING      */
/*!**********************************************************************
 *
 * DESCRIPTION
 *
 *    This is a simple utility based on the original fitsgrab program.
 *    It is designed to extract a FITS image from the ZWO camera
 *    server.  Unlike with fitsgrab the entire fits image can be
 *    downloaded as binary data.
 *
 *********************************************************************!*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <ctype.h>

#include "sockio/sockclnt.h" 
#include "fh/fh.h"
#include "fhreg/general.h"
#include "cfht/cfht.h"
#include "cli/cli.h"
#include "ss/ss_define.h"
#include "ssapi/ss_api.h"
#include "ssapi/ss_error.h"

#define BINARY_CMD "binary"
#define IMAGE_CMD "image"

#define SS_PATH "/i/dualcam/visible"
#define SS_IP_ADDRESS SS_PATH"/ipAddress"
#define SS_PORT SS_PATH"/port"
#define SS_LASTIMAGE SS_PATH"/lastImage"
#define SOCKET_TIMEOUT 150
//#define SS_ASIVA_STATUS "/i/zwocam/systemStatus"
//#define SS_ASIVA_AVAILABILITY "/i/ASIVA/systemAvailability"

//#define SS_ASIVA_STATUS "/i/asiva/brock/systemStatus"
//#define SS_ASIVA_AVAILABILITY "/i/asiva/brock/systemAvailability"

static void
usage(void)
{
   fprintf(stderr, "usage: zwograb [rootdir=] [etime=<sec>] [gain=[0..510]] > stdout\n");
}


int
main(int argc, char* argv[])
{
   sockclnt_t *sock;
   const char *reply;
   char buf[8192];
   char ip_address[80];
   char port[20];
   int count = 1;
   int i = 1;
   int nbytes, nread, nwrite;
   char status;
   char file_name[255];
   char file_directory[255];
   int fd;
   char *args[argc];
  
   time_t rawtime;
   struct tm *time_info;
   char time_buffer[255];
   char date_buffer[255];
   char system_status[255];
   char system_availability[255];

   if (argc < 4) {
      usage();
      exit(EXIT_FAILURE);
   }
   
   // connect to status server
   if (ssLogon(argv[0]) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) unable to connect to Status Server: %s",
		__FILE__, __LINE__, ssGetStrError());
      exit(EXIT_FAILURE);
   }

   /* Check if an image should be taken */
/*   if(strncmp(system_status, "SLEEP", sizeof(system_status)) == 0 || 
      strncmp(system_availability, "ONLINE", sizeof(system_availability)) != 0) {
      // system unavailable. Exit
      printf("System Offline. No image taken");
      exit(EXIT_SUCCESS);
   }
*/ 
   //parse argv for rootdir=
   count = 1;
   while(count < argc){
      char *arg = argv[count];
      char *val = "rootdir=";
      char *tmp = strstr(arg, val);
      if(tmp != NULL) {
         break;
      }
      count++;
   }
   if(count == 5) {
      //roodir= not found
      usage();
      exit(EXIT_FAILURE);
   }
   else {
      strncpy(file_directory, argv[count] + 8, 255);
   }
   

   strncat(file_directory, "/", 255);
   time( &rawtime );
   time_info = localtime( &rawtime );
   strftime(time_buffer,255,"%H_%M_%S", time_info);
   strftime(date_buffer,255,"%Y_%m_%d", time_info);
   strncat(file_directory, date_buffer, 255);

   mkdir(file_directory, S_IRWXU | S_IRWXG | S_IRWXO);
   //strncat(file_directory, "/", 255);
   strncpy(file_name, time_buffer, 255);
   strncat(file_name, ".fits", 255);
   //mkdir(file_directory, S_IRWXU | S_IRWXG | S_IRWXO);

   strncat(date_buffer, "_", 255);
   strncat(date_buffer, time_buffer, 255);

   //strncpy(file_name, file_directory, 255);
   //strncat(file_name, "/", 255);
   //strncpy(file_name, "image.fits", 255);
   printf("Directory created: %s\n", file_directory);
   printf("Writing to: %s\n", file_name);


   //create args removing rootdir
   for(i = 0; i < argc; i++){
      args[i] = argv[i];
   }
   for(i = count; i < argc - 1; i++){
      args[i] = argv[i + 1];
   }


   /* Set up the CFHT logging calls */
   cfht_logv(CFHT_MAIN, CFHT_LOG_ID,
             cfht_basename((char *)NULL, argv[0], (char *)NULL));
   cfht_logv(CFHT_MAIN, CFHT_START,
             "%s", cfht_argsToString(argc, argv));

   /*
    * Extract the hostname and IP address for the ZWO camera to
    * connect to
    */
   if (ssGetString(SS_IP_ADDRESS, ip_address, sizeof(ip_address)-1) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) unable to extract ZWO camera server IP address"
		" from %s : %s", __FILE__, __LINE__, SS_IP_ADDRESS,
		ssGetStrError());
      exit(EXIT_FAILURE);
   }
   if (ssGetString(SS_PORT, port, sizeof(port)-1) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) unable to extract ZWO server port from"
		" %s : %s", __FILE__, __LINE__, SS_PORT, ssGetStrError());
      exit(EXIT_FAILURE);
   }
   // initialize last image timestamp on ss
   if (ssTouchObject(SS_LASTIMAGE,
                     "Image Timestamp") != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
                "(%s:%d) ssTouchObject of %s failed: %s",
                __FILE__, __LINE__, SS_LASTIMAGE,
                ssGetStrError());
   } 

   /*
    * Connect to the camera server.
    */
   snprintf(buf, sizeof(buf)-1, "%s:%s", ip_address, port);
   if ((sock = sockclnt_create(buf, SOCKET_TIMEOUT)) == NULL) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) unable to connect to ZWO camera server at %s",
		__FILE__, __LINE__, buf);
      exit(EXIT_FAILURE);
   }
   cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	     "(%s:%d) connected to ZWO camera server at %s",
	     __FILE__, __LINE__, buf);

   /*
    * Send parameters given on the command line (see usage).
    */
   count = 1;
   while (count < argc -  1) {
      char *equal;
      char *arg = NULL;
      char *p;

      if (arg) {
	 free(arg);
      }
      arg = strdup(args[count++]);
      /*
       * Take the equal sign that is part of a command and convert it to be
       * a space before the command is sent on to the server.
       */
      equal = strchr(arg, '=');
      if (equal) {
	 *equal=' ';
      }

      /*
       * Substitute commas for spaces in case of the raster spec
       */
      while ((p = strchr(arg, ',')) != NULL) {
	 *p = ' ';
      }

      /*
       * Send the command on to the ZWO camera server
       */
      sockclnt_send(sock, arg);
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) send '%s' to the ZWO camera server",
		__FILE__, __LINE__, arg);
      reply = sockclnt_recv(sock);
      if (!reply || *reply == '!') {
	 cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		   "(%s:%d) error received from the ZWO camera server."
		   "  Response = '%s'", __FILE__, __LINE__, reply);
	 exit(EXIT_FAILURE);
      }
   }

   /*
    * Start the exposure.
    */
   sockclnt_send(sock, IMAGE_CMD);
   cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	     "(%s:%d) send '%s' to the ZWO camera server",
	     __FILE__, __LINE__, IMAGE_CMD);
   sockclnt_set_mode(sock, SOCKCLNT_MODE_BINARY);
   reply = sockclnt_recv(sock);

   /*
    * Read the reply to get the resulting image size.
    */
   if (!reply || sscanf(reply, "%c %d", 
			&status, &nbytes) != 2) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) error received from the ZWO camera server."
		"  Response = '%s'", __FILE__, __LINE__, reply);
      exit(EXIT_FAILURE);
   }
   //open file for writing
   fd = open(file_name, O_CREAT | O_RDWR | O_TRUNC,
             S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP | S_IWOTH);
   /*
    * Read the image data from the server.
    */
   count = 0;
   while (count < nbytes) {
      nread = nbytes;
      if (nread > sizeof(buf)) {
	 nread = sizeof(buf);
      }
      nread = read(sock->fd, buf, nread);
#ifdef DEBUG
      printf("logonly: (%s:%d) bytes read = %d, total = %d\n", 
	     __FILE__, __LINE__, nread, count + nread);
#endif
      switch (nread) {
	 case 0: 
	    cfht_logv(CFHT_MAIN, CFHT_ERROR,
		      "(%s:%d) unexpected EOF on socket read",
		      __FILE__, __LINE__);
	    exit(EXIT_FAILURE);
	 case -1:
	    if (errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN) {
	       usleep(1000);
	       continue;
	    }
	    cfht_logv(CFHT_MAIN, CFHT_ERROR,
		      "(%s:%d) read from socket failed : %s"
		      " (errno=%d)", __FILE__, __LINE__, strerror(errno),
		      errno);
	    exit(EXIT_FAILURE);
	 default:
	    count += nread;
	    nwrite = 0;

	    while (nwrite < nread) {
	       nwrite = write(fd, buf, nread);
	       switch (nwrite) {
		  case 0:
		     cfht_logv(CFHT_MAIN, CFHT_ERROR, 
			       "(%s:%d) unexpected EOF on file"
			       " write to stdout", __FILE__, __LINE__);
		     exit(EXIT_FAILURE);
		  case -1:
			if(errno == EAGAIN) {
			usleep(1000);
			continue;
		     }
		     cfht_logv(CFHT_MAIN, CFHT_ERROR,
			       "(%s:%d) write to stdout failed : %s"
			       " (errno=%d)", __FILE__, __LINE__, 
			       strerror(errno), errno);
		     exit(EXIT_FAILURE);
		  default:
		     ;
	       }
	    }
      }
   }
   close(fd);

   if (ssPutString(SS_LASTIMAGE, date_buffer) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
                "(%s:%d) ssPutString on %s with %s failed: %s",
             __FILE__, __LINE__, SS_LASTIMAGE, date_buffer, ssGetStrError());
      exit(EXIT_FAILURE);
   }

   /*
    * Disconnect from the server if no more commands to send.
    */
   sockclnt_send(sock, "quit");
   reply = sockclnt_recv(sock);
   sockclnt_destroy(sock);

   exit(EXIT_SUCCESS);
}
