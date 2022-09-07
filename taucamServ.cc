/* -*- c-file-style: "Ellemtel" -*- */
/* Copyright (C) 2022   Canada-France-Hawaii Telescope Corp.          */
/* This program is distributed WITHOUT any warranty, and is under the */
/* terms of the GNU General Public License, see the file COPYING      */
/*!**********************************************************************
 *
 * DESCRIPTION
 *
 *    Server process which connects to a FLIR camera via a
 *    thermalgrabber USB interface and listens for client requests to 
 *    receive images.
 *
 *********************************************************************!*/
#include <mutex>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <math.h>
#include <netdb.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <fcntl.h>
#include <pthread.h>

#include "cli/cli.h"
#include "fh/fh.h"
#include "fhreg/general.h"
#include "fhreg/cfh_telescope.h"
#include "sockio/sockserv.h"
#include "sockio/sockclnt.h"
#include "ss/linked_list.h"
#include "ssapi/ss_api.h"
#include "ssapi/ss_error.h"
#include "thermalgrabber/thermalgrabber.h"

#define TAUSERV_PORT "916" /* Port name or number on which to listen */
#define READOUT_TIMEOUT 15 /* Abort readout if client disappears for 15 sec. */
#define EXPOSE_TIMEOUT 5   /* Amount beyond etime to wait before timeout */
#define SEND_BUF_SIZE 5000 /* Number of pixels to send at a time */

#define SOCKSERV_IDLE_POLL_INTERVAL 1   /* Corresponds to 1 second */
#define MAX_EXPOSURE_DELAY 100 /* Maximum exposure time */
#define MAX_RETRIES 2          /* Maximum number of retries */
#define NULL_DBL -9999.9
#define MIN_GAIN 0
#define MAX_GAIN 510

#define SQR(X) ((X)*(X))

#define IMAGE_CMD "IMAGE"
#define ETIME_CMD "ETIME"
#define GAIN_CMD "GAIN"
#define QUIT_CMD "QUIT"
#define BYE_CMD "BYE"
#define EXIT_CMD "EXIT"
#define LOGOUT_CMD "LOGOUT"
#define PASS_CHAR '.'
#define FAIL_CHAR '!'
#define TEMP_FILE "/tmp/image.fits"
#define GAIN_AUTO_STRING "AUTO"
#define GAIN_HIGH_STRING "HIGH"
#define GAIN_LOW_STRING "LOW"
#define GAIN_MANUAL_STRING "MANUAL"

#define SS_PATH "/i/dualcam/IR"
#define SS_ETIME SS_PATH"/etime"
#define SS_GAIN SS_PATH"/gain"
#define SS_HOSTNAME SS_PATH"/hostname"
#define SS_IPADDRESS SS_PATH"/ipAddress"
#define SS_PORT SS_PATH"/port"
#define SS_SERVER_RUNNING SS_PATH"/serverRunning"
#define SS_DOME_AZ "/t/status/domeAz"

#define SS_TEMP SS_PATH"/temperature"
#define SS_PRES SS_PATH"/pressure"
#define SS_HUMID SS_PATH"/humidity"

#define FLIR_MODEL "FLIR TAU 2 640x512"
#define PIXEL_SIZE 17
#define MIN_ETIME 0.1
#define MAX_ETIME 600
#define DEFAULT_ETIME 1

/*
 * Fast element swap
 */
#define ELEM_SWAP(a,b) {register uint16_t t=(a);(a)=(b);(b)=t; }

#define FLIP_X

#define DEBUG


typedef enum {
   GAIN_AUTO,
   GAIN_HIGH,
   GAIN_LOW,
   GAIN_MANUAL
} gain_t;

/*
 * Structure used to specify server specific information.
 */
typedef struct {
      linked_list *client_list;
      sockserv_t *tau_serv;
      ThermalGrabber *tgr;
      int serv_done;
      gain_t gain;
      double etime;
      unsigned int width;
      unsigned int height;
      int *stack_data;
      unsigned int frame_count;
      double exp_start_ts;
} server_info_t;


/*
 * Per-client information.  Multiple clients can stay connected to the
 * server at once, although this is typically not the case.
 */
typedef struct
{
   char *hostname;
   unsigned char remote_ip[4];
   time_t connect_ts;
   int send_data;
   int data_count;
   int total_count;
   unsigned char *image_data;
   unsigned int frame_count;
} client_info_t;


/*
 * Server information structure instance
 */
static server_info_t *serv_info = NULL;

/*
 * Set up a C++ mutex since the libthermalgrabber C++ library uses a 
 * background thread when images are available
 */
std::mutex mtx;
//static pthread_mutex_t lock;

/*
 * Utility function to return the IP address associated with the "eth0"
 * Ethernet interface.  Since this is currently running on a Raspberry PI
 * the "eth0" interface is the correct interface to use for the IP address
 */
static const char *
getIPAddress(void) {

   struct ifaddrs *interface_array = NULL;
   struct ifaddrs *if_ptr = NULL;
   static char ip_address[20];

   /* 
    * Get the IP address
    */
   if (getifaddrs(&interface_array) != 0) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) getifaddrs call failed: %s (errno=%d)",
		__FILE__, __LINE__, strerror(errno), errno);
      return NULL;
   }

   /* 
    * Iterate through the IP addresses and return the address associated 
    * with eth0
    */
   if_ptr = interface_array;
   while (if_ptr) {
      if (if_ptr->ifa_addr && if_ptr->ifa_addr->sa_family == AF_INET) {
	 struct sockaddr_in *p_addr = (struct sockaddr_in *)if_ptr->ifa_addr;
	 if (strcasecmp(if_ptr->ifa_name, "eth0") == 0) {
	    snprintf(ip_address, sizeof(ip_address), "%s", 
		     inet_ntoa(p_addr->sin_addr));
	    cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		      "(%s:%d) IP address = %s", __FILE__, __LINE__, 
		      ip_address);
	    return ip_address;
	 }
      }
      if_ptr = if_ptr->ifa_next;
   }

   freeifaddrs(interface_array);

   return NULL;
}


/*
 * Perform a touch initialization of all the state parameters that are read
 * from the Status Server upon startup and updated in the Status Server 
 * whenever they are changed.
 */
static PASSFAIL
ssTouchState(void) {

   /*
    * Perform a touch of the current gain
    */
   if (ssTouchObject(SS_GAIN,
		     "Gain Value") != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY, 
		"(%s:%d) ssTouchObject of %s failed: %s", 
		__FILE__, __LINE__, SS_GAIN, 
		ssGetStrError());
      return FAIL;
   }

   /*
    * Perform a touch of the current exposure time
    */
   if (ssTouchObject(SS_ETIME,
		     "Exposure Time") != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY, 
		"(%s:%d) ssTouchObject of %s failed: %s", 
		__FILE__, __LINE__, SS_ETIME, 
		ssGetStrError());
      return FAIL;
   }

   /*
    * Perform a touch of the hostname
    */
   if (ssTouchObject(SS_HOSTNAME, "Server Host Name") != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) ssTouchObject of %s failed: %s",
		__FILE__, __LINE__, SS_HOSTNAME, ssGetStrError());
      return FAIL;
   }

   /*
    * Perform a touch of the IP address
    */
   if (ssTouchObject(SS_IPADDRESS, "Server IP Address") != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) ssTouchObject of %s failed: %s",
		__FILE__, __LINE__, SS_IPADDRESS, ssGetStrError());
      return FAIL;
   }

   /*
    * Perform a touch of the port
    */
   if (ssTouchObject(SS_PORT, "Command Server Port Number") != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) ssTouchObject of %s failed: %s",
		__FILE__, __LINE__, SS_PORT, ssGetStrError());
      return FAIL;
   }

   /*
    * Perform a touch of the server running flag
    */
   if (ssTouchObject(SS_SERVER_RUNNING, 
		     "Command Server Running Flag") != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) ssTouchObject of %s failed: %s",
		__FILE__, __LINE__, SS_SERVER_RUNNING, ssGetStrError());
      return FAIL;
   }

   return PASS;
}


/* 
 * Advance past leading whitespace in a string 
 */
static char *
ltrim(char *str) {
   char *p;
   
   if (str == NULL) 
      return NULL;
   
   for (p = str; (*p && isspace(*p)); p++)
      ;

   return p;
}


/* 
 * Trim off trailing whitespace in a string 
 */
static char *
rtrim(char *str) {
   int i;

   if (str != NULL) {
      for (i = strlen(str) - 1; (i >= 0 && isspace(str[i])); i--)
	 ;
      str[++i] = '\0';
   }   

   return str;
}


/* 
 * Trim off all leading and trailing whitespace from a string 
 */
static char *
trim(char *str) {
   return rtrim(ltrim(str));
}


/*
 * Get the current clock timestamp
 */
static double
getClockTime(void) {

   struct timespec ts;

   if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY, 
		"(%s:%d) unable to get clock timestamp : %s (errno=%d)",
		__FILE__, __LINE__, strerror(errno), errno);
      return 0;
   }
   return (double)(ts.tv_sec + ts.tv_nsec / 1000000000.0);
}     


/*
 * Perform a fast median calculation of the elements within an array.  The
 * following routine is faster than using the c qsort() function call.
 * The array that is passed in will be modified so it should not have the
 * original contents if there is a desire to have these preserved.
 * The size of the array is specified by 'n' and the multiplying factor
 * is specified by 'm'
 */
unsigned short
medianCalculation(unsigned short *arr, unsigned int n, unsigned int m) 
{

   int low, high; 
   int median; 
   int middle, ll, hh;

   /* Reduce the number of elements in the array by the multiplying factor */
   n /= m;

   low = 0; high = n - 1; median = (low + high) / 2; 
   for (;;) { 
      
      if (high <= low) { /* One element only */ 
	 return arr[median * m];
      }      

      if (high == low + 1) { /* Two elements only */ 
	 if (arr[low * m] > arr[high * m]) 
	    ELEM_SWAP(arr[low * m], arr[high * m]);
	 return arr[median * m];
      } 
      
      /* Find median of low, middle and high items; swap into position low */ 
      middle = (low + high) / 2; 
      if (arr[middle * m] > arr[high * m]) 
	 ELEM_SWAP(arr[middle * m], arr[high * m]);
      if (arr[low * m] > arr[high * m]) 
	 ELEM_SWAP(arr[low * m], arr[high * m]); 
      if (arr[middle * m] > arr[low * m]) 
	 ELEM_SWAP(arr[middle * m], arr[low * m]); 
			
      /* Swap low item (now in position middle) into position (low + 1) */ 
      ELEM_SWAP(arr[middle * m], arr[(low+1) * m]);
      
      /* Nibble from each end towards middle, swapping items when stuck */ 
      ll = low + 1; 
      hh = high; 
      for (;;) { 
	 do 
	    ll++; 
	 while (arr[low * m] > arr[ll * m]); 
	 do 
	    hh--; 
	 while (arr[hh * m] > arr[low * m]); 
	 
	 if (hh < ll) 
	    break;
	 
	 ELEM_SWAP(arr[ll * m], arr[hh * m]);
      } 
      
      /* Swap middle item (in position low) back into correct position */ 
      ELEM_SWAP(arr[low * m], arr[hh * m]);
	 
	 /* Re-set active partition */ 
      if (hh <= median) 
	 low = ll;
      if (hh >= median) 
	 high = hh - 1; 
   } 
} 


/*
 * Handle callbacks for image data received from the camera.  Take the 
 * data from the camera and if the clock time indicates that we're in an
 * exposing state, add the image to the stack
 */
static void 
callbackTauImage(TauRawBitmap &bitmap, void *caller) {

   unsigned short *image_copy;
   unsigned short median;

   /*
    * Check if a new image needs to be allocated
    */
   if (serv_info->stack_data == NULL) {
      serv_info->frame_count = 0;
      serv_info->width = bitmap.width;
      serv_info->height = bitmap.height;
      serv_info->stack_data = (int *)cli_malloc(serv_info->width * 
						serv_info->height * 
						sizeof(int));
      memset(serv_info->stack_data, 0, 
	     serv_info->width * serv_info->height * sizeof(int));
   }

   /*
    * Make sure that the size of the bitmap is consistent
    */
   if ((bitmap.width != serv_info->width) ||
       (bitmap.height != serv_info->height)) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) inconsistency image size consistency"
		" width = %d vs %d, height = %d vs %d", 
		__FILE__, __LINE__, bitmap.width, serv_info->width,
		bitmap.height, serv_info->height);
      return;
   }	    
   
   /*
    * If this is not received within the exposure sequence, reset the
    * image buffer and count.
    */
   if (serv_info->exp_start_ts == 0) {
      serv_info->frame_count = 0;
      memset(serv_info->stack_data, 0, 
	     serv_info->width * serv_info->height * sizeof(int));
   }

   /*
    * Make a copy of the image and determine the median background
    */
   image_copy = (unsigned short *)cli_malloc(serv_info->width * 
					     serv_info->height * 
					     sizeof(unsigned short));
   memcpy(image_copy, bitmap.data, serv_info->width * serv_info->height * 
	  sizeof(unsigned short));
   median = medianCalculation(image_copy, serv_info->width * serv_info->height,
			      1);
   free(image_copy);
      
   /*
    * Add the median subtracted pixels to the stacked image
    */
//   pthread_mutex_lock(&lock);
   mtx.lock();
   for (unsigned int i = 0; i < serv_info->width * serv_info->height; i++) {
      //serv_info->stack_data[i] += bitmap.data[i] - median;
      serv_info->stack_data[i] = bitmap.data[i];
   }
   serv_info->frame_count++;
   mtx.unlock();

//   pthread_mutex_unlock(&lock);
}


/*
 * Set a new camera gain
 */
static void
applyGain(gain_t gain) {

   char gain_string[20];

   switch(gain) {
      case GAIN_AUTO:
	 serv_info->tgr->setGainMode(thermal_grabber::Automatic);
	 snprintf(gain_string, sizeof(gain_string)-1, "AUTO");
	 break;
      case GAIN_LOW:
	 serv_info->tgr->setGainMode(thermal_grabber::LowGain);
	 snprintf(gain_string, sizeof(gain_string)-1, "LOW");
	 break;
      case GAIN_HIGH:
	 serv_info->tgr->setGainMode(thermal_grabber::HighGain);
	 snprintf(gain_string, sizeof(gain_string)-1, "HIGH");
	 break;
      case GAIN_MANUAL:
	 serv_info->tgr->setGainMode(thermal_grabber::Manual);
	 snprintf(gain_string, sizeof(gain_string)-1, "MANUAL");
	 break;
   }
   if (ssPutString(SS_GAIN, gain_string) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) ssPutString of %s with %s failed: %s",
		__FILE__, __LINE__, SS_GAIN, gain_string, ssGetStrError());
   }
}


/*
 * Initialize the camera connection.
 */
static PASSFAIL
initCameraConnection(void) {

   cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	     "Initializing Tau camera connection");

   /* 
    * Set up a connection to the camera.  The way the API in the SDK is set
    * up this request won't fail
    */
   serv_info->tgr = new ThermalGrabber(callbackTauImage, NULL);

   /*
    * Apply the default gain which is automatic
    */
   applyGain(serv_info->gain);

   return PASS;
}


/*
 * Take a pointer to image data and create a FITS image using this data
 * and send it to the specified file descriptor
 */
static PASSFAIL
writeFITSImage(client_info_t *cinfo, int fd) 
{
   HeaderUnit hu;
   time_t date = time(NULL);
   char fitscard[FH_MAX_STRLEN];
   fh_result fh_error;
   unsigned short *image;
   unsigned int i;
   int min_val;
   char dome_az[255];
   char temp[255];
   char pres[255];
   char humid[255];

   // Get SS Data 
   if (ssGetString(SS_DOME_AZ, dome_az, sizeof(dome_az) - 1) == FAIL) {
      fprintf(stderr, "ssGet `%s' failed: %s\n", SS_DOME_AZ, ssGetStrError());
      exit(EXIT_FAILURE);
   }
   if (ssGetString(SS_TEMP, temp, sizeof(temp) - 1) == FAIL) {
      fprintf(stderr, "ssGet `%s' failed: %s\n", SS_TEMP, ssGetStrError());
      exit(EXIT_FAILURE);
   }
   if (ssGetString(SS_PRES, pres, sizeof(pres) - 1) == FAIL) {
      fprintf(stderr, "ssGet `%s' failed: %s\n", SS_PRES, ssGetStrError());
      exit(EXIT_FAILURE);
   }
   if (ssGetString(SS_HUMID, humid, sizeof(humid) - 1) == FAIL) {
      fprintf(stderr, "ssGet `%s' failed: %s\n", SS_HUMID, ssGetStrError());
      exit(EXIT_FAILURE);
   }


   /*
    * Make sure there is actually an image to send back to the client
    */
   if ((serv_info->frame_count == 0) || (serv_info->stack_data == NULL)) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) image data is not available to send to the"
		" client", __FILE__, __LINE__);
      return FAIL;
   }

   /*
    * Create the header unit
    */
   hu = fh_create();

   /*
    * Populate the headers 
    */
   fh_set_bool(hu, FH_AUTO, "SIMPLE", FH_TRUE, "Standard FITS");
   fh_set_int(hu, FH_AUTO, "BITPIX", 16, "16-bit data");
   fh_set_int(hu, FH_AUTO, "NAXIS", 2, "Number of axes");
   fh_set_int(hu, FH_AUTO, "NAXIS1", serv_info->width,
	      "Number of pixel columns");
   fh_set_int(hu, FH_AUTO, "NAXIS2", serv_info->height,
	      "Number of pixel rows");
   fh_set_int(hu, FH_AUTO, "PCOUNT", 0, "No 'random' parameters");
   fh_set_int(hu, FH_AUTO, "GCOUNT", 1, "Only one group");
   strftime(fitscard, sizeof (fitscard) - 1, "%Y-%m-%dT%T", gmtime(&date));
   fh_set_str(hu, FH_AUTO, "DATE", fitscard, "UTC Date of file creation");
   strftime(fitscard, sizeof (fitscard) - 1, "%a %b %d %H:%M:%S %Z %Y",
	    localtime(&date));
   fh_set_str(hu, FH_AUTO, "HSTTIME", fitscard, "Local time in Hawaii");
   fh_set_flt(hu, FH_AUTO, "UNIXTIME", getClockTime(), 
	      13, "Fractional UNIX timestamp when image was taken");
   fh_set_str(hu, FH_AUTO, "ORIGIN", "CFHT",
	      "Canada-France-Hawaii Telescope");
   fh_set_flt(hu, FH_AUTO, "BZERO", 32768.0, 6, "Zero factor");
   fh_set_flt(hu, FH_AUTO, "BSCALE", 1.0, 2, "Scale factor");
   fh_set_flt(hu, FH_AUTO, "ETIME", serv_info->etime, 2, "Exposure time");
   fh_set_int(hu, FH_AUTO, "STACKCNT", serv_info->frame_count, 
	      "Number of stacked subframes");
   switch (serv_info->gain) {
      case GAIN_AUTO:
	 fh_set_str(hu, FH_AUTO, "GAIN", "AUTO", "Camera Gain");
	 break;
      case GAIN_HIGH:
	 fh_set_str(hu, FH_AUTO, "GAIN", "HIGH", "Camera Gain");
	 break;
      case GAIN_LOW:
	 fh_set_str(hu, FH_AUTO, "GAIN", "LOW", "Camera Gain");
	 break;
      case GAIN_MANUAL:
	 fh_set_str(hu, FH_AUTO, "GAIN", "MANUAL", "Camera Gain");
	 break;
   }
   fh_set_str(hu, FH_AUTO, "DOMEAZ", dome_az, "Dome Azimuth");
   fh_set_str(hu, FH_AUTO, "TEMP", temp, "Enclosure Temperature");
   fh_set_str(hu, FH_AUTO, "PRESSURE", pres, "Enclosure Pressure");
   fh_set_str(hu, FH_AUTO, "HUMID", humid, "Enclosure Humidity");
   fh_set_str(hu, FH_AUTO, "CAMMODEL", FLIR_MODEL, "Camera Model");

   /* 
    * Write out the FITS header 
    */
   if ((fh_error = fh_write(hu, fd)) != FH_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) unable to write FITS header"
		" (fh_error = %d)", __FILE__, __LINE__,
		fh_error);
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY, 
		"%s (errno=%d)", strerror(errno), errno);
      fh_destroy(hu);
      return FAIL;
   }

//   pthread_mutex_lock(&lock);
   mtx.lock();

   /*
    * Find the minimum pixel value of the stacked image and adjust the 
    * threshold of the image by this amount
    */
   min_val = 65535;
   for (i = 0; i < serv_info->width * serv_info->height; i++) {
      if (serv_info->stack_data[i] < min_val) {
	 min_val = serv_info->stack_data[i];
      }
   }

   /*
    * Take the stacked image and set it up to be saved with an offset bias
    */
   image = (unsigned short *)cli_malloc(serv_info->width * serv_info->height * 
					sizeof(unsigned short));

   /*
    * Adjust the image by the minimum stack value
    */
   for (i = 0; i < serv_info->width * serv_info->height; i++) {
      if (serv_info->stack_data[i] - min_val > 65535) {
	 image[i] = 65535;
      }
      else {
	 image[i] = serv_info->stack_data[i] - min_val;
      }
   }
   serv_info->exp_start_ts = 0;
   mtx.unlock();
//   pthread_mutex_unlock(&lock);
   
   /*
    * Write out the image data
    */
   if ((fh_error = fh_write_padded_image(hu, fd,
					 (unsigned short *)image, 
					 serv_info->width * serv_info->height *
					 sizeof(unsigned short), 
					 FH_TYPESIZE_16U)) 
       != FH_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) unable to write FITS image data"
		" (fh_error = %d)", __FILE__, __LINE__,
		fh_error);
      free(image);
      //fh_destroy(hu);
      return FAIL;
   }
   
   /*
    * Free up the memory for the FITS header and image
    */
   fh_destroy(hu);
   free(image);
   
   return PASS;
}


/*
 * Take an image and save the contents of the buffer into memory where it 
 * can be sent to the client at the first opportunity.
 */
static void
takeImage(client_info_t *cinfo, char *buffer)
{
   struct stat file_stat;
   double stop_ts;
   int fd;
   int read_count;
   int count;

   /*
    * Open up a temporary file to write the FITS image to
    */
   if ((fd = open(TEMP_FILE, O_CREAT | O_RDWR | O_TRUNC,
		  S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP | S_IWOTH))
       == -1) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) unable to open temporary file %s.  %s (errno=%d)",
		__FILE__, __LINE__, TEMP_FILE, strerror(errno), errno);
      sprintf(buffer,
	      "%c %s \"Unable to save temporary file on the camera"
	      " server\"", FAIL_CHAR, IMAGE_CMD);
      cfht_logv(CFHT_MAIN, CFHT_DEBUG, "(%s:%d) SEND> %s", __FILE__, __LINE__,
		buffer);
      return;
   }

   /*
    * Make sure that a valid exposure time has been established
    */
   if (serv_info->etime < MIN_ETIME || serv_info->etime > MAX_ETIME) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) exposure time must be set before triggering an"
		" image", __FILE__, __LINE__);
      sprintf(buffer,
	      "%c %s \"Exposure time isn't set\"", FAIL_CHAR, IMAGE_CMD);
      return;
   }

   /*
    * Start the capture of the exposure and sleep until the exposure is done
    */
   mtx.lock();
   serv_info->frame_count = 0;
   memset(serv_info->stack_data, 0, 
	  serv_info->width * serv_info->height * sizeof(int));
   serv_info->exp_start_ts = getClockTime();
   stop_ts = getClockTime() + serv_info->etime;
   mtx.unlock();
   do {
      usleep(100);

      /*
       * Handle an exception case where the exposure is timing out
       */
      if ((serv_info->frame_count == 0) && 
	  (serv_info->exp_start_ts + EXPOSE_TIMEOUT < getClockTime())) {
	 cfht_logv(CFHT_MAIN, CFHT_ERROR,
		   "(%s:%d) exposure timeout without receiving any frames"
		   " from the camera", __FILE__, __LINE__);
	 sprintf(buffer, "%c %s \"Exposure timeout\"", FAIL_CHAR, IMAGE_CMD);
	 cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		   "(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
	 close(fd);
	 return;
      }
   } while ((getClockTime() < stop_ts) || (serv_info->frame_count == 0));

   /*
    * After the sleep a stacked image should be available.  Create a FITS
    * image from the pixel data and save it in the file
    */
   if (writeFITSImage(cinfo, fd) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) unable to create FITS file", __FILE__, __LINE__);
      sprintf(buffer, "%c %s \"Unable to save temporary file on the guide"
	      " server\"", FAIL_CHAR, IMAGE_CMD);
      cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		"(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
      close(fd);
      serv_info->exp_start_ts = 0;
      return;
   }

   /*
    * Reset the exposure start time.  This will reset the stacking of images
    */
   serv_info->exp_start_ts = 0;

   /*
    * Take the FITS image that was written to disk and determine the size of
    * it.
    */
   if (fstat(fd, &file_stat) == -1) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) unable to retrieve the size of the temporary FITS"
		" file", __FILE__, __LINE__);
      sprintf(buffer, "%c %s \"Unable to save temporary file on the guide"
	      " server\"", FAIL_CHAR, IMAGE_CMD);
      cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		"(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
      close(fd);
      return;
   }

   /*
    * Copy the image data to the client structure
    */
   cinfo->image_data = (unsigned char *)cli_malloc(file_stat.st_size);
   read_count = 0;
   lseek(fd, 0, SEEK_SET);
   while (read_count < file_stat.st_size) {
      count = read(fd, cinfo->image_data + read_count,
		   file_stat.st_size - read_count);
      if (count < 0) {
	 cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		   "(%s:%d) unable to retrieve FITS image data from tmpfs"
		   " file.  %s (errno=%d)", __FILE__, __LINE__,
		   strerror(errno), errno);
	 sprintf(buffer, "%c %s \"Unable to retrieve FITS image data"
		 " from tmpfs\"", FAIL_CHAR, IMAGE_CMD);
	 cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		   "(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
	 close(fd);
	 return;
      }
      read_count += count;
   }

   /*
    * Set up the flags to trigger data being sent
    */
   cinfo->send_data = 1;
   cinfo->data_count = 0;
   cinfo->total_count = file_stat.st_size;

   /*
    * If we made it this far, send back a response with the number of bytes
    * of binary data that can be expected to be received from the server.
    */
   sprintf(buffer, "%c %ld", PASS_CHAR, file_stat.st_size);
   close(fd);

   return;
}


/*
 * Handle a new client connection
 */
static void*
client_add(unsigned char remote_ip[4])
{
   client_info_t *cinfo;
   struct hostent *hp;	  /* Host entry returned from gethostbyaddr */

   /*
    * Allocate memory for the client data structure
    */
   cinfo = (client_info_t *)cli_malloc(sizeof(*cinfo));
   memset(cinfo, 0, sizeof(client_info_t));
   memcpy(cinfo->remote_ip, remote_ip, sizeof (cinfo->remote_ip));

   /* 
    * Determine the hostname from the IP address 
    */
   hp = gethostbyaddr(cinfo->remote_ip, sizeof(struct in_addr), AF_INET);
   if (hp == NULL) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	    "(%s:%d) gethostbyaddr call failed", __FILE__, __LINE__);
      cinfo->hostname = cli_strdup("UNKNOWN");
   }
   else {
      cinfo->hostname = cli_strdup(hp->h_name);
   }

   /* 
    * Store the time the client connected to the Control Server.  This
    * is logged when a request is made to list the connected clients.
    */
   time(&(cinfo->connect_ts));

   /* 
    * Add the client to the linked list of clients connected to the 
    * server.
    */
   appendDataToList(cinfo, serv_info->client_list, cli_malloc);

   return cinfo;
}


/*
 * Handle a client disconnecting
 */
static void
client_del(void *client, char *buffer)
{
   client_info_t *cinfo = (client_info_t *)client;

   /*
    * Free up the memory allocated by the client
    */
   if (cinfo->hostname != NULL) {
      free(cinfo->hostname);
   }
   if (cinfo->image_data != NULL) {
      free(cinfo->image_data);
   }
   free(cinfo);
}


/*
 * Handle a command from a client
 */
static void
client_recv(void *client, char *buffer)
{
   int cargc = 0;		/* number of arguments found in args */
   static char **cargv = NULL;	/* arguments in split form */
   client_info_t *cinfo = (client_info_t *)client;
   char *buf_p;
   char *p;

   /* 
    * Advance past the command for argument parsing purposes.
    */
   buf_p = trim(buffer);
   p = strchr(buf_p, ' ');
   if (p == NULL) {

      /* ----------------------------------------------------
       * Handle all commands which don't have any parameters
       * ----------------------------------------------------*/

      /*
       * Handle the various ways the client can disconnect from the 
       * Control Server.
       */
      if (!strcasecmp(buf_p, QUIT_CMD) ||
	  !strcasecmp(buf_p, BYE_CMD) ||
	  !strcasecmp(buf_p, EXIT_CMD) || 
	  !strcasecmp(buf_p, LOGOUT_CMD)) {
	 buffer[0] = '\0';
	 return;
      }

      /*
       * Handle an image request from the client
       */
      if (!strcasecmp(buf_p, IMAGE_CMD)) {
	 takeImage((client_info_t *)cinfo, buffer);
	 return;
      }


      /*
       * Handle commands that were received without parameters specified.
       */
      if (!strcasecmp(buf_p, ETIME_CMD)) {
	 sprintf(buffer, "%c %s \"Argument not specified\"", 
		 FAIL_CHAR, ETIME_CMD);
	 cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		   "(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
	 return;
      }

      /*
       * Handle commands that were received without parameters specified.
       */
      if (!strcasecmp(buf_p, GAIN_CMD)) {
	 sprintf(buffer, "%c %s \"Argument not specified\"", 
		 FAIL_CHAR, GAIN_CMD);
	 cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		   "(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
	 return;
      }

      /*
       * If we made it this far, this is an unrecognized command request
       * from the client which doesn't have parameters.
       */
      sprintf(buffer, "%c \"Syntax error\"", FAIL_CHAR);
      cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		"(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
      return;
   }

   /* ---------------------------------------------
    * Handle all commands which contain parameters
    * ---------------------------------------------*/
   *p = '\0';

   /*
    * Free up memory associated with arguments previously parsed 
    */
   if (cargv != NULL) {
      cli_argv_free(cargv);
   }
   cargv = cli_argv_quoted(&cargc, ++p);

   /*
    * Handle an exposure time request from a client
    */
   if (!strcasecmp(buf_p, ETIME_CMD)) {
      char *stop_at = NULL;   /* Location at which strtod may stop */
      double etime;
      
      /*
       * Make sure that an exposure time argument
       */
      if (cargc != 1) {
	 sprintf(buffer, "%c %s \"Invalid argument specified\"", 
		 FAIL_CHAR, ETIME_CMD);
	 cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		   "(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
	 return;
      }

      /*
       * Make sure the exposure time is valid
       */
      errno = 0;
      etime = strtod(cargv[0], &stop_at);
      
      /* Make sure the exposure time is valid */
      if ((errno == ERANGE) || (errno == EINVAL) || (*stop_at != '\0')) {
	 sprintf(buffer, "%c %s \"Invalid argument specified\"", 
		 FAIL_CHAR, ETIME_CMD);
	 cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		   "(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
	 return;
      }
      if (etime < MIN_ETIME || etime > MAX_ETIME) {
	 sprintf(buffer, "%c %s \"Invalid exposure time specified\"", 
		 FAIL_CHAR, ETIME_CMD);
	 cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		   "(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
	 return;
      }

      /*
       * Reset the exposure time fields
       */
      serv_info->etime = etime;
      serv_info->frame_count = 0;
      serv_info->exp_start_ts = 0;
      if (cinfo->image_data != NULL) {
	 free(cinfo->image_data);
	 cinfo->image_data = NULL;
      }
      if (ssPutPrintf(SS_ETIME, "%.3f", serv_info->etime) != PASS) {
	 cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		   "(%s:%d) ssPutPrintf of %s with %.3f failed: %s",
		   __FILE__, __LINE__, SS_ETIME, serv_info->etime, 
		   ssGetStrError());
      }

      sprintf(buffer, "%c %s", PASS_CHAR, ETIME_CMD);
      cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		"(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
      return;
   }

   /*
    * Handle a gain request from a client
    */
   if (!strcasecmp(buf_p, GAIN_CMD)) {

      /*
       * Make sure that an argument was specified
       */
      if (cargc != 1) {
	 sprintf(buffer, "%c %s \"Invalid argument specified\"", 
		 FAIL_CHAR, GAIN_CMD);
	 cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		   "(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
	 return;
      }

      /*
       * Set the gain depending upon the argument specified
       */
      if (!strcasecmp(cargv[0], GAIN_AUTO_STRING)) {
	 serv_info->gain = GAIN_AUTO;
      } else if (!strcasecmp(cargv[0], GAIN_HIGH_STRING)) {
	 serv_info->gain = GAIN_HIGH;
      } else if (!strcasecmp(cargv[0], GAIN_LOW_STRING)) {
	 serv_info->gain = GAIN_LOW;
      } else if (!strcasecmp(cargv[0], GAIN_MANUAL_STRING)) {
	 serv_info->gain = GAIN_MANUAL;
      }
      else {
	 sprintf(buffer, "%c %s \"Invalid gain argument specified\"", 
		 FAIL_CHAR, GAIN_CMD);
	 cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		   "(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
	 return;
      }

      /*
       * Apply the new gain
       */
      applyGain(serv_info->gain);
      sprintf(buffer, "%c %s", PASS_CHAR, GAIN_CMD);
      cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		"(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
      return;
   }

   /*
    * If we made it this far, this is an unrecognized command request
    * from the client.
    */
   sprintf(buffer, "%c \"Syntax error\"", FAIL_CHAR);
   cfht_logv(CFHT_MAIN, CFHT_DEBUG,
	     "(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
}


/*
 * Send out binary image data to a client
 */
static void
client_send_binary(void *client, char *buffer, int *len)
{
   client_info_t *cinfo = (client_info_t *)client;

   /*
    * Check whether this client has any data to be sent.
    */
   if (cinfo->send_data) {

      int send_count;

      /* 
       * If all the data has been sent, send out a final summary 
       */
      if (cinfo->data_count == cinfo->total_count) {
	 *len = 0;
	 cinfo->send_data = 0;
	 cinfo->data_count = 0;
	 if (cinfo->image_data != NULL) {
	    free(cinfo->image_data);
	    cinfo->image_data = NULL;
	 }
	 return;
      }

      /*
       * Send out data to the client
       */
      send_count = cinfo->total_count - cinfo->data_count;
      if (send_count > SEND_BUF_SIZE) {
	 send_count = SEND_BUF_SIZE;
      }
      memcpy(buffer, cinfo->image_data + cinfo->data_count, send_count);
      *len = send_count;
      cinfo->data_count += send_count;
   }
}


/*
 * Handle a cleanup of the socket resources and make sure the shutter is 
 * closed.
 */
static void
cleanup(void) {

   cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	     "(%s:%d) cleanup()", __FILE__, __LINE__);

   /*
    * Mark the server as no longer running
    */
   if (ssPutBoolean(SS_SERVER_RUNNING, FALSE) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) ssPutBoolean on %s failed: %s",
		__FILE__, __LINE__, SS_SERVER_RUNNING, ssGetStrError());
      exit(EXIT_FAILURE);
   }
   
   /*
    * Clean up the listening socket 
    */
   if (serv_info->tau_serv != NULL) {
      sockserv_destroy(serv_info->tau_serv);
      serv_info->tau_serv = NULL;
   }

   exit(EXIT_SUCCESS);
}


int
main(int argc, const char* argv[])
{
   char hostname[255];
   const char *ip_address;

   /*
    * Set up the environment variable used by the cfht_log system to 
    * determine whether debug messages will be logged.
    */
//   setenv("CFHTDEBUG", "On", 1);

   /*
    * Initialize the CFHT logging stuff.
    */
   cfht_log(CFHT_MAIN, CFHT_LOG_ID, argv[0]);
   cfht_logv(CFHT_MAIN, CFHT_START, "%s", argv[0]);

#ifdef OLD_CODE
   /*
    * Initialize the mutex
    */
   if (pthread_mutex_init(&lock, NULL) != 0) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) mutex init failed",
		__FILE__, __LINE__);
      exit(EXIT_FAILURE);
   }
#endif

   /*
    * Set up all memory allocations to retry if memory is available.  This 
    * has the added advantage that there is no need to change for memory
    * allocation failures.  If memory isn't available, it will hang until
    * memory becomes available.
    */
   cli_malloc_retry(TRUE);

   /* 
    * Allocate and initialize the server information structure 
    */
   serv_info = (server_info_t *)cli_malloc(sizeof(server_info_t));
   memset(serv_info, 0, sizeof(server_info_t));
   
   /*
    * Create a linked list to hold client entries
    */
   serv_info->client_list = createList(cli_malloc);

   /* 
    * Connect to the Status Server
    */
   while (ssLogon(argv[0]) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_WARN,
		"(%s:%d): connection to Status Server failed...retry in"
		" progress: %s", __FILE__, __LINE__, ssGetStrError());
      sleep(60);
   }

   /*
    * Perform a status server touch on all those config parameters that may
    * be changed by the program.
    */
   if (ssTouchState() != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) unable to perform a touch on all the status server"
		" object potentially modified by this program",
		__FILE__, __LINE__);
      exit(EXIT_FAILURE);
   }

   /*
    * Set a default exposure time of 1 second
    */
   serv_info->etime = DEFAULT_ETIME;
   if (ssPutPrintf(SS_ETIME, "%.3f", serv_info->etime) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) ssPutPrintf of %s with %.3f failed: %s",
		__FILE__, __LINE__, SS_ETIME, serv_info->etime, 
		ssGetStrError());
   }
   
   /*
    * Handle termination and interrupt signals
    */
   cli_signal(SIGTERM, cleanup);
   cli_signal(SIGINT, cleanup);

   /*
    * Try to initialize the camera connection.  If the connection can not
    * be established, exit.
    */
   if (initCameraConnection() != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR, 
		"(%s:%d) Unable to establish connection to"
		" the FLIR Tau camera", __FILE__, __LINE__);
      exit(EXIT_FAILURE);
   }

   /* 
    * Set up the server parameters
    */
   serv_info->tau_serv = sockserv_create(TAUSERV_PORT);
   if (!(serv_info->tau_serv)) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR, 
		"(%s:%d) Unable to establish listening port"
		" for the server", __FILE__, __LINE__);
      exit(EXIT_FAILURE);
   }

   /*
    * Establish the hostname where the server is currently running
    */
   if (gethostname(hostname, sizeof(hostname)-1) != 0) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) uanble to get the hostname: %s (errno=%d)",
		__FILE__, __LINE__, strerror(errno), errno);
      exit(EXIT_FAILURE);
   }
   if ((ip_address = getIPAddress()) == NULL) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) unable to get the IP address",
		__FILE__, __LINE__);
      exit(EXIT_FAILURE);
   }
   if (ssPutString(SS_HOSTNAME, hostname) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) ssPutString on %s with %s failed: %s",
	     __FILE__, __LINE__, SS_HOSTNAME, hostname, ssGetStrError());
      exit(EXIT_FAILURE);
   }
   if (ssPutString(SS_IPADDRESS, ip_address) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) ssPutString on %s with %s failed: %s",
	     __FILE__, __LINE__, SS_IPADDRESS, ip_address, ssGetStrError());
      exit(EXIT_FAILURE);
   }
   if (ssPutString(SS_PORT, TAUSERV_PORT) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) ssPutString on %s with %s failed: %s",
		__FILE__, __LINE__, SS_PORT, TAUSERV_PORT, ssGetStrError());
      exit(EXIT_FAILURE);
   }

   /*
    * Mark the server as running
    */
   if (ssPutBoolean(SS_SERVER_RUNNING, TRUE) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) ssPutBoolean on %s failed: %s",
		__FILE__, __LINE__, SS_SERVER_RUNNING, ssGetStrError());
      exit(EXIT_FAILURE);
   }

   /* 
    * Cleanup camera and socket resources before exiting 
    */
   atexit(cleanup);

   /*
    * Register callback functions 
    */
   serv_info->tau_serv->client_add_hook = client_add;
   serv_info->tau_serv->client_del_hook = client_del;
   serv_info->tau_serv->client_recv_hook = client_recv;
   serv_info->tau_serv->client_send_binary_hook = client_send_binary;

   cfht_logv(CFHT_MAIN, CFHT_LOGONLY, 
	     "(%s:%d) Camera is ready to answer requests",
	     __FILE__, __LINE__);

   /* 
    * Go through a loop processing any commands sent by the client.
    */
   for (;;) {

      cli_signal_block(SIGTERM);
      cli_signal_block(SIGINT);

      sockserv_run(serv_info->tau_serv, SOCKSERV_IDLE_POLL_INTERVAL);

      cli_signal_unblock(SIGTERM);
      cli_signal_unblock(SIGINT);
   }
   exit(EXIT_SUCCESS);
}
