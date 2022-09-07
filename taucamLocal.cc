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
#include "ssapi/ss_error.h"
#include "ssapi/ss_api.h"
#include "ss/linked_list.h"
#include "thermalgrabber/thermalgrabber.h"

#define TAUSERV_PORT "915" /* Port name or number on which to listen */
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

#define SS_PATH "/i/taucam"
#define SS_ETIME SS_PATH"/etime"
#define SS_GAIN SS_PATH"/gain"
#define SS_HOSTNAME SS_PATH"/hostname"
#define SS_IPADDRESS SS_PATH"/ipAddress"
#define SS_PORT SS_PATH"/port"
#define SS_SERVER_RUNNING SS_PATH"/serverRunning"

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
 * get current clock ms
 */
static int 
getClockTimeMS(void){
   struct timespec ts;
   if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY, 
                "(%s:%d) unable to get clock timestamp : %s (errno=%d)",
                __FILE__, __LINE__, strerror(errno), errno);
      return 0;
   }
   return (int)(ts.tv_nsec / 1000000);
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
      serv_info->stack_data[i] += bitmap.data[i] - median;
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
 * send a packet of data to the camera server
 * see Tau 2 software IDD
 * packet protocol (table 3.2)
 */
static PASSFAIL
send_packet(char cmd, char *arg, unsigned int data_len)
{
   serv_info->tgr->sendCommand(cmd, arg, data_len);
}

/*
 * get temperature data from camera core
 *
 */
static PASSFAIL
get_core_tmp()
{
   constexpr char GET_DIGITAL_OUTPUT_MODE[2] = {0x20};

   send_packet(GET_DIGITAL_OUTPUT_MODE[0],const_cast<char*>(&GET_DIGITAL_OUTPUT_MODE[1]),sizeof(GET_DIGITAL_OUTPUT_MODE));
/*
   constexpr char args[1] = {0x0A};
   send_packet(0x20, const_cast<char*>(&args[0]), 0);*/
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
  
   get_core_tmp(); 
   //enable TLinear mode
   serv_info->tgr->enableTlinearLowResolution(); 
   return PASS;
}


/*
 * Take a pointer to image data and create a FITS image using this data
 * and send it to the specified file descriptor
 */
static PASSFAIL
writeFITSImage(int fd, int *image) 
{
   HeaderUnit hu;
   time_t date = time(NULL);
   char fitscard[FH_MAX_STRLEN];
   fh_result fh_error;
   unsigned int i;

   /*
    * Create the header unit
    */
   hu = fh_create();

   /*
    * Populate the headers 
    */
   fh_set_bool(hu, FH_AUTO, "SIMPLE", FH_TRUE, "Standard FITS");
   fh_set_int(hu, FH_AUTO, "BITPIX", 32, "32-bit data");
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

   
   /*
    * Write out the image data
    */
   if ((fh_error = fh_write_padded_image(hu, fd,
					 image, 
					 serv_info->width * serv_info->height *
					 sizeof(int), 
					 sizeof(int))) 
       != FH_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) unable to write FITS image data"
		" (fh_error = %d)", __FILE__, __LINE__,
		fh_error);
      free(image);
      fh_destroy(hu);
      return FAIL;
   }
   
   /*
    * Free up the memory for the FITS header
    */
   fh_destroy(hu);
   return PASS;
}

/*
 * Take the differential of two images by subtracting them pixel by pixel
 *
 */
static PASSFAIL
takeDifferential(const char *fileName, unsigned short *img_1, unsigned short *img_2, char *buffer)
{
   int *img_diff;
   unsigned int i;
   int fd;
 
   img_diff = (int *)cli_malloc(serv_info->width * serv_info->height * 
					sizeof(int));
   for (i = 0; i < serv_info->width * serv_info->height; i++) {
      img_diff[i] = (int)(img_2[i] - img_1[i]); 
   }
   /*
    * Open up a temporary file to write the FITS image to
    */
   if ((fd = open(fileName, O_CREAT | O_RDWR | O_TRUNC,
                  S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP | S_IWOTH))
       == -1) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) unable to open file %s.  %s (errno=%d)",
		__FILE__, __LINE__, fileName, strerror(errno), errno);
      sprintf(buffer,
	      "%c %s \"Unable to save temporary file on the camera"
	      " server\"", FAIL_CHAR, IMAGE_CMD);
      cfht_logv(CFHT_MAIN, CFHT_DEBUG, "(%s:%d) SEND> %s", __FILE__, __LINE__,
		buffer);
      free(img_diff);
      return NULL;
   }


   if (writeFITSImage(fd, img_diff) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) unable to create FITS file", __FILE__, __LINE__);
      sprintf(buffer, "%c %s \"Unable to save temporary file on the guide"
	      " server\"", FAIL_CHAR, IMAGE_CMD);
      cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		"(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
      close(fd);
      free(img_diff);
      serv_info->exp_start_ts = 0;
      return FAIL;
   }
   close(fd);
   free(img_diff);
   return PASS;
}



/*
 * Take an image and save the contents of the buffer into memory where it 
 * can be sent to the client at the first opportunity.
 */
unsigned short *
takeImage(const char *fileName, char *buffer)
{
   unsigned short *image;
   struct stat file_stat;
   double stop_ts;
   int fd;
   int read_count;
   int count;
   int min_val;
   int i;
   //char *file_name = (char *)malloc(sizeof(fileName));
   //if(strcpy(file_name, fileName) == NULL){
   //   strcpy(file_name, TEMP_FILE);
   //}

   /*
    * Open up a temporary file to write the FITS image to
    */
   if ((fd = open(fileName, O_CREAT | O_RDWR | O_TRUNC,
                  S_IRUSR | S_IRGRP | S_IROTH | S_IWUSR | S_IWGRP | S_IWOTH))
       == -1) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) unable to open file %s.  %s (errno=%d)",
		__FILE__, __LINE__, fileName, strerror(errno), errno);
      sprintf(buffer,
	      "%c %s \"Unable to save temporary file on the camera"
	      " server\"", FAIL_CHAR, IMAGE_CMD);
      cfht_logv(CFHT_MAIN, CFHT_DEBUG, "(%s:%d) SEND> %s", __FILE__, __LINE__,
		buffer);
      //free(file_name);
      return NULL;
   }
   //free(file_name);

   /*
    * Make sure that a valid exposure time has been established
    */
   if (serv_info->etime < MIN_ETIME || serv_info->etime > MAX_ETIME) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) exposure time must be set before triggering an"
		" image", __FILE__, __LINE__);
      sprintf(buffer,
	      "%c %s \"Exposure time isn't set\"", FAIL_CHAR, IMAGE_CMD);
      return NULL;
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
	 return NULL;
      }
   } while ((getClockTime() < stop_ts) || (serv_info->frame_count == 0));

   /*
    * After the sleep a stacked image should be available.  Create a FITS
    * image from the pixel data and save it in the file
    */


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
	 image[i] = 0.4 * (serv_info->stack_data[i] - min_val);
      }
   }
   serv_info->exp_start_ts = 0;
   mtx.unlock();
//   pthread_mutex_unlock(&lock);
   
   //create copy of image as int *
   int *img_cpy = (int *)cli_malloc(serv_info->width * serv_info->height * sizeof(int));
   for(i = 0; i < serv_info->width * serv_info->height; i++) {
      img_cpy[i] = (int)image[i];
   }
   if (writeFITSImage(fd, img_cpy) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) unable to create FITS file", __FILE__, __LINE__);
      sprintf(buffer, "%c %s \"Unable to save temporary file on the guide"
	      " server\"", FAIL_CHAR, IMAGE_CMD);
      cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		"(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
      close(fd);
      free(img_cpy);
      serv_info->exp_start_ts = 0;
      return NULL;
   }
   free(img_cpy);

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
      return NULL;
   }


   /*
    * If we made it this far, send back a response with the number of bytes
    * of binary data that can be expected to be received from the server.
    */
   //sprintf(buffer, "%c %ld", PASS_CHAR, file_stat.st_size);
   close(fd);

   return image;
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

   exit(EXIT_SUCCESS);
}

gain_t
str_to_gain(const char *gain_str)
{
   int gain;
   
   //determine gain ENUM value
   if(strncmp("AUTO", gain_str, 10) == 0){
      gain = 0;
   }
   else if(strncmp("HIGH", gain_str, 10) == 0){
      gain = 1;
   }
   if(strncmp("LOW", gain_str, 10) == 0){
      gain = 2;
   }
   else if(strncmp("MANUAL", gain_str, 10) == 0){
      gain = 3;
   }
   else{
      //default to AUTO
      gain = 0;
   }
   gain_t gain_val = (gain_t)gain;

   return gain_val;
}




/*
 * command line args:
 * 1 - exposure time (in ms)
 * 2 - gain (AUTO, HIGH, LOW, MANUAL). Defaults to AUTO if invalid input
 * 3 - Differential time (in s)(if 0 - take a single image)
 */
int
main(int argc, const char* argv[])
{
   /*
    * set variables
    */
   char *buf;
   char hostname[255];
   const char *ip_address;
   unsigned short *image_1;
   unsigned short *image_2;
   char image_1_name[255];
   char image_2_name[255];
   char image_dif_name[255];
   double diff_stop_ts;

   char file_name[255]; 
   double exposure_time = (double)((atoi(argv[1])) / 1000);
   const char *gain_str = argv[2];
   double diff_time = (double)(atoi(argv[3]));
   gain_t gain_val = str_to_gain(gain_str);
   
   //timestamp directory 
   time_t rawtime;
   struct tm *time_info;
   char time_buffer[255];
   char date_buffer[255];
   char ms_buffer[255];
   int ms_stamp = getClockTimeMS();
   sprintf(ms_buffer, "%d", ms_stamp);

   time( &rawtime );
   time_info = localtime( &rawtime );
   strftime(time_buffer,255,"%H_%M_%S", time_info);
   strftime(date_buffer,255,"%Y_%m_%d", time_info);
   mkdir(date_buffer, S_IRWXU | S_IRWXG | S_IRWXO);
   strncat(date_buffer, "/", 255);
   strncat(date_buffer, time_buffer, 255);
   mkdir(date_buffer, S_IRWXU | S_IRWXG | S_IRWXO);
   strncpy(file_name, date_buffer, 255);
   strncat(file_name, "/", 255);
   strncat(file_name, ms_buffer, 255);
   strncat(file_name, "_IR", 255);
   printf("%s\n", file_name);

   strncpy(image_1_name, file_name, 255);
   strncpy(image_2_name, file_name, 255);
   strncpy(image_dif_name, file_name, 255);
   
   if(diff_time != 0){
      strncat(image_1_name, "_IM1", 255);
   }

   strncat(image_2_name, "_IM2", 255);
   strncat(image_dif_name, "_diff", 255);
   strncat(image_1_name, ".fits", 255);
   strncat(image_2_name, ".fits", 255);
   strncat(image_dif_name, ".fits", 255);
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
   
   serv_info->gain = gain_val;
   serv_info->etime = exposure_time;

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

   if(serv_info->etime == NULL){
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
   }
   
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
    * Cleanup camera and socket resources before exiting 
    */
   atexit(cleanup);

   cfht_logv(CFHT_MAIN, CFHT_LOGONLY, 
	     "(%s:%d) Camera is ready to answer requests",
	     __FILE__, __LINE__);

   //serv_info->tgr->doFFC();





   //take image 1
   image_1 = takeImage(image_1_name, buf);


   //take image 2 if in differential mode
   if(diff_time != 0){
      //stall for differential image
      diff_stop_ts = getClockTime() + diff_time;
      printf("stalling for image 2");
      while (getClockTime() < diff_stop_ts);
      image_2 = takeImage(image_2_name, buf);
      printf("got image 2");
      takeDifferential(image_dif_name, image_1, image_2, buf);
   }

   
   printf("Finished");
   //free(image_1);
   //free(image_2);
   printf(buf);
   exit(EXIT_SUCCESS);
}
