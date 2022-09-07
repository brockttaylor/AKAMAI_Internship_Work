/* -*- c-file-style: "Ellemtel" -*- */
/* Copyright (C) 2022   Canada-France-Hawaii Telescope Corp.          */
/* This program is distributed WITHOUT any warranty, and is under the */
/* terms of the GNU General Public License, see the file COPYING      */
/*!**********************************************************************
 *
 * DESCRIPTION
 *
 *    Server process which connects to a ZWO camera via USB and 
 *    listens for client requests to receive images.
 *
 *********************************************************************!*/
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

#include "cli/cli.h"
#include "fh/fh.h"
#include "fhreg/general.h"
#include "fhreg/cfh_telescope.h"
#include "sockio/sockserv.h"
#include "sockio/sockclnt.h"
#include "ss/linked_list.h"
#include "ssapi/ss_api.h"
#include "ssapi/ss_error.h"

#include "ASICamera2.h"

#define ZWOSERV_PORT "915" /* Port name or number on which to listen */
#define READOUT_TIMEOUT 15 /* Abort readout if client disappears for 15 sec. */
#define EXPOSE_TIMEOUT 30   /* Amount beyond etime to wait before timeout */
#define SEND_BUF_SIZE 5000 /* Number of pixels to send at a time */

#define SOCKSERV_IDLE_POLL_INTERVAL 1   /* Corresponds to 1 second */
#define MAX_EXPOSURE_DELAY 100 /* Maximum exposure time */
#define MAX_RETRIES 2          /* Maximum number of retries */
#define NULL_DBL -9999.9
#define MIN_GAIN 0
#define MAX_GAIN 510

#define SQR(X) ((X)*(X))

#define IMAGE_CMD "IMAGE"
#define PASS_CHAR '.'
#define FAIL_CHAR '!'
#define TEMP_FILE "/tmp/image.fits"

#define SS_PATH "/i/dualcam/visible"
#define SS_ETIME SS_PATH"/etime"
#define SS_GAIN SS_PATH"/gain"
#define SS_HOSTNAME SS_PATH"/hostname"
#define SS_IPADDRESS SS_PATH"/ipAddress"
#define SS_PORT SS_PATH"/port"
#define SS_SERVER_RUNNING SS_PATH"/serverRunning"
#define SS_DOME_AZ "/t/status/domeAz"

#define ZWO_MODEL "ZWO ASI178MM"
#define CCD_SENSOR "Sony CMOS IMX178"
#define PIXEL_SIZE 2.4

#define FLIP_X

#define DEBUG


/*
 * Structure used to specify server specific information.
 */
typedef struct {
      linked_list *client_list;
      sockserv_t *zwo_serv;
      ASI_CAMERA_INFO *asi_camera_info;
      int serv_done;
      double etime;	/* Exposure time in seconds */
      int gain;
      int image_width;
      int image_height;
      int frame_sequence;
      double exp_start_ts;
      double exp_readout_done_ts;
      double exp_cycle_time;
      time_t last_exp_completion;
      unsigned char *image_data;
      char *response_buffer;
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
   int width;
   int height;
   unsigned char *image_data;
} client_info_t;


/*
 * Server information structure instance
 */
static server_info_t *serv_info = NULL;


/*
 * Case insensitive string occurance search
 */
static char *
stristr(const char *str, const char *pattern) 
{
   char *pptr, *sptr, *start;
   unsigned int slen, plen;
   
   for (start = (char *)str, 
	   pptr = (char *)pattern, 
	   slen = strlen(str), 
	   plen = strlen(pattern);
	
	/* while string length not shorter than pattern length */
	slen >= plen;
	start++, slen--) {
      
      /* find start of pattern in string */
      while (toupper(*start) != toupper(*pattern)) {
	 start++;
	 slen--;
	 
	 /* if pattern longer than string */
	 if (slen < plen)
	    return(NULL);
      }
      sptr = start;
      pptr = (char *)pattern;
      
      while (toupper(*sptr) == toupper(*pptr)) {
	 sptr++;
	 pptr++;
	 
	 /* if end of pattern then pattern was found */
	 if (*pptr == '\0')
	    return (start);
      }
   } 
   return(NULL);
}


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
 * Initialize the camera connection.
 */
static PASSFAIL
initCameraConnection(void) {

   int rc;
   int num_cameras;
   int num_controls;
   ASI_CONTROL_CAPS *asi_control_caps;
   int i;

   cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	     "Initializing ZWO camera connection");

   /* 
    * Read the number of connected cameras and make sure only one camera
    * is found.
    */
   num_cameras = ASIGetNumOfConnectedCameras();
   if (num_cameras != 1) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) Unable to detect a ZWO camera", __FILE__, __LINE__);
      return FAIL;
   }

   /*
    * Get the camera properties
    */
   serv_info->asi_camera_info 
      = (ASI_CAMERA_INFO *)cli_malloc(sizeof(ASI_CAMERA_INFO));
   if ((rc = ASIGetCameraProperty(serv_info->asi_camera_info, 
				  0)) != ASI_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) ASIGetCameraProperty() failed: rc=%d", 
		__FILE__, __LINE__, rc);
      return FAIL;
   }

   /*
    * Set the width and height
    */
   serv_info->image_width = serv_info->asi_camera_info->MaxWidth;
   serv_info->image_height = serv_info->asi_camera_info->MaxHeight;

   cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	     "(%s:%d) width=%d, height=%d",
	     __FILE__, __LINE__, serv_info->image_width, 
	     serv_info->image_height);
   cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	      "(%s:%d) bit depth=%d",
	      __FILE__, __LINE__, serv_info->asi_camera_info->BitDepth);

   /*
    * Open the camera connection
    */
   if ((rc = ASIOpenCamera(serv_info->asi_camera_info->CameraID)) 
       != ASI_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) ASIOpenCamera() failed: rc=%d",
		__FILE__, __LINE__, rc);
      return FAIL;
   }

   /*
    * Initialize the camera 
    */
   if ((rc = ASIInitCamera(serv_info->asi_camera_info->CameraID)) 
       != ASI_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) ASIInitCamera() failed: rc=%d",
		__FILE__, __LINE__, rc);
      return FAIL;
   }

   /*
    * Set the ROI format
    */
   if ((rc = ASISetROIFormat(serv_info->asi_camera_info->CameraID,
			     serv_info->image_width, 
			     serv_info->image_height,
			     1, ASI_IMG_RAW16)) != ASI_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) ASISetROIFormat() failed: rc=%d", 
		__FILE__, __LINE__, rc);
      return FAIL;
   }

   /*
    * Get the ROI format
    */
   ASI_IMG_TYPE img_type;
   int width, height, bin;
   if ((rc = ASIGetROIFormat(serv_info->asi_camera_info->CameraID,
			     &width, &height, &bin, 
			     &img_type)) != ASI_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) ASIGetROIFormat() failed: rc=%d", 
		__FILE__, __LINE__, rc);
      return FAIL;
   }
   cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	     "(%s:%d) ROI width=%d, ROI height=%d, ROI bin=%d, type=%d",
	     __FILE__, __LINE__, width, height, bin, img_type);

   /*
    * Set the high speed mode to be off
    */
   if ((rc = ASISetControlValue(serv_info->asi_camera_info->CameraID,
				ASI_HIGH_SPEED_MODE, 0,
				ASI_FALSE)) != ASI_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) Unable to set high speed mode to be 0: rc=%d",
		__FILE__, __LINE__, rc);
      return FAIL;
   }

   /*
    * Get the camera properties
    */
   num_controls = 0;
   if (ASIGetNumOfControls(serv_info->asi_camera_info->CameraID, 
			   &num_controls) != ASI_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) Error getting number of controls of camera #0",
		__FILE__, __LINE__);
      return FAIL;
   }
   
   asi_control_caps 
      = (ASI_CONTROL_CAPS *)cli_malloc(sizeof(ASI_CONTROL_CAPS));
   for (i = 0; i < num_controls; i++) {
      if ((rc = ASIGetControlCaps(serv_info->asi_camera_info->CameraID, i, 
				  asi_control_caps)) == ASI_SUCCESS) {
	 
	 /*
	  * Print out the camera properties
	  */
	 cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		   "(%s:%d) Property %s: [%ld, %ld] = %ld%s - %s",
		   __FILE__, __LINE__,
		   asi_control_caps->Name,
		   asi_control_caps->MinValue,
		   asi_control_caps->MaxValue,
		   asi_control_caps->DefaultValue,
		   asi_control_caps->IsWritable == 1 ? " (set)" : "",
		   asi_control_caps->Description
	    );
      }
   }

   return PASS;
}


/*
 * Set the exposure time
 */
static PASSFAIL
com_etime(const char *arg)
{
   double new_etime;
   char *s = cli_arg1(arg);
   char *colon = strchr(s, ':');
   double tgt_min = 0.001, tgt_max = 600.0, tgt_stp = 0.001;

   cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	     "(%s:%d) com_etime (args=%s)", __FILE__, __LINE__, arg);

   if (colon) {
      new_etime = atoi(s) * 60.0 + atof(colon+1);
   }
   else {
      new_etime = atof(s);
   }
   free(s);

   if (new_etime != 0) {

      /*
       * Round to nearest tgt_stp
       */
      new_etime += tgt_stp/2.;
      new_etime = (int)(new_etime / tgt_stp) * tgt_stp;
      if (new_etime > tgt_max) new_etime = tgt_max;
      if (new_etime < tgt_min) new_etime = tgt_min;

      if (fabs(serv_info->etime - new_etime) > 0.0001) {
	 serv_info->etime = new_etime;
      }
   }

   if (ssPutPrintf(SS_ETIME, 
		   "%.4f", serv_info->etime) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY, 
		"(%s:%d) ssPutPrintf of %s failed: %s", 
		__FILE__, __LINE__, SS_ETIME, 
		ssGetStrError());
   }

   cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	     "(%s:%d) etime set to %.3f", 
	     __FILE__, __LINE__, serv_info->etime);

   sprintf(serv_info->response_buffer, ". etime %.3f", serv_info->etime);

   return PASS;
}


/*
 * Set the gain value
 */
static PASSFAIL
com_gain(const char *arg)
{
   char *s = cli_arg1(arg);
   int gain = 0;

   cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	     "(%s:%d) com_gain (args=%s)", __FILE__, __LINE__, arg);

   gain = atoi(s);
   if (gain < MIN_GAIN || gain > MAX_GAIN) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) Invalid gain specified: %s should be between %d and"
		" %d", __FILE__, __LINE__, s, MIN_GAIN, MAX_GAIN);
      free(s);
      sprintf(serv_info->response_buffer, "! gain \"invalid value\"");
      return PASS;
   }
   free(s);
   serv_info->gain = gain;

   /*
    * Store the new gain value in the Status Server
    */
   if (ssPutPrintf(SS_GAIN, 
		   "%d", serv_info->gain) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY, 
	 	"(%s:%d) ssPutPrintf of %s failed: %s", 
		__FILE__, __LINE__, SS_GAIN, 
		ssGetStrError());
   }

   cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	     "(%s:%d) gain set to %d", 
	     __FILE__, __LINE__, serv_info->gain);

   sprintf(serv_info->response_buffer, ". gain %d", serv_info->gain);

   return PASS;
}


/*
 * Take a pointer to image data and create a FITS image using this data
 * and send it to the specified file descriptor
 */
static PASSFAIL
writeFITSImage(unsigned short *image_p, int fd) 
{

   HeaderUnit hu;
   time_t date = time(NULL);
   char fitscard[FH_MAX_STRLEN];
   struct timeval tv;
   struct timezone tz;
   fh_result fh_error;
   char dome_az[255];

   // Get dome azimuth 
   if (ssGetString(SS_DOME_AZ, dome_az, sizeof(dome_az) - 1) == FAIL) {
      fprintf(stderr, "ssGet `%s' failed: %s\n", SS_DOME_AZ, ssGetStrError());
      exit(EXIT_FAILURE);
   }
   /*
    * Create the header unit
    */
   hu = fh_create();

   /*
    * Populate the headers 
    */
   fh_set_bool(hu, FH_AUTO, "SIMPLE", FH_TRUE, "Standard FITS");
   fh_set_int(hu,  FH_AUTO, "BITPIX", 16,"16-bit data");
   fh_set_int(hu,  FH_AUTO, "NAXIS",  2, "Number of axes");
   fh_set_int(hu,  FH_AUTO, "NAXIS1", serv_info->image_width, 
	 "Number of pixel columns");
   fh_set_int(hu,  FH_AUTO, "NAXIS2", serv_info->image_height, 
	 "Number of pixel rows");
   fh_set_int(hu,  FH_AUTO, "PCOUNT", 0, "No 'random' parameters");
   fh_set_int(hu,  FH_AUTO, "GCOUNT", 1, "Only one group");

   strftime(fitscard, sizeof(fitscard)-1, "%Y-%m-%dT%T", gmtime(&date));
   fh_set_str(hu, FH_AUTO, "DATE", fitscard, "UTC Date of file creation");
   strftime(fitscard, sizeof(fitscard)-1, "%a %b %d %H:%M:%S %Z %Y", 
	 localtime(&date));
   fh_set_str(hu, FH_AUTO, "HSTTIME", fitscard, "Local time in Hawaii");
   gettimeofday(&tv, &tz);
   fh_set_flt(hu, FH_AUTO, "UNIXTIME", getClockTime(), 
	 13, "Fractional UNIX timestamp when image was taken");
   fh_set_str(hu, FH_AUTO, "ORIGIN", "CFHT", "Canada-France-Hawaii Telescope");
   fh_set_str(hu, FH_AUTO, "INSTRUME", "ZWOCam", "Instrument Name");
   fh_set_flt(hu, FH_AUTO, "BZERO", 32768.0, 6,	"Zero factor");
   fh_set_flt(hu, FH_AUTO, "BSCALE", 1.0, 2, "Scale factor");

   fh_set_str(hu, FH_AUTO, "DOMEAZ", dome_az, "Dome Azimuth");
   fh_set_str(hu, FH_AUTO, "CAMMODEL", ZWO_MODEL, "Camera Model");
   fh_set_str(hu, FH_AUTO, "CCDNAME", CCD_SENSOR, "CCD Sensor");
   fh_set_flt(hu, FH_AUTO, "ETIME", serv_info->etime, 5, 
	      "Integration time");
   fh_set_int(hu, FH_AUTO, "GAIN", serv_info->gain, 
	      "Camera Gain [0..510]");
   fh_set_flt(hu, FH_AUTO, "PIXSIZE", PIXEL_SIZE, 5, "Pixel size (micron)");
   fh_set_int(hu, FH_AUTO, "SEQNUM", ++(serv_info->frame_sequence), 
	 "Frame sequence number");

   /* 
    * Add space to allow for additional headers 
    */
   if ((fh_error = fh_reserve(hu, 220)) != FH_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
	      "(%s:%d) unable to add padding to FITS header:"
		" (fh_error=%d)", __FILE__, __LINE__, fh_error);
      fh_destroy(hu);
      return FAIL;
   }

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
					 (unsigned short *)image_p, 
					 serv_info->image_width *
					 serv_info->image_height *
					 sizeof(uint16_t), FH_TYPESIZE_16U)) 
       != FH_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) unable to write FITS image data"
		" (fh_error = %d)", __FILE__, __LINE__,
		fh_error);
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
 * Take an image and save the contents of the buffer into memory where it 
 * can be sent to the client at the first opportunity.
 */
static void
takeImage(client_info_t *cinfo, char *buffer)
{
   struct stat file_stat;
   ASI_EXPOSURE_STATUS asi_exp_status;
   int fd;
   int rc;
   int read_count;
   int count;
   int size;

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
    * Check if there is an exposure already in progress.  If so, wait until 
    * the exposure is done.
    */
   do {
      ASIGetExpStatus(serv_info->asi_camera_info->CameraID,
		      &asi_exp_status);
      usleep(5000);
   } while (asi_exp_status == ASI_EXP_WORKING);

   /*
    * Set the exposure time
    */
   if ((rc = ASISetControlValue(serv_info->asi_camera_info->CameraID,
				ASI_EXPOSURE, 
				(long)(serv_info->etime * 1000000),
				ASI_FALSE)) != ASI_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) Unable to set exposure time to %f seconds: rc=%d",
		__FILE__, __LINE__, serv_info->etime, rc);
      sprintf(buffer,
	      "%c %s \"Unable to set exposure time\"", FAIL_CHAR, IMAGE_CMD);
      return;
   }

   /*
    * Set the gain value
    */
   if ((rc = ASISetControlValue(serv_info->asi_camera_info->CameraID,
				ASI_GAIN, serv_info->gain,
				ASI_FALSE)) != ASI_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) Unable to set gain to be %d: rc=%d",
		__FILE__, __LINE__, serv_info->gain, rc);
      sprintf(buffer,
	      "%c %s \"Unable to set gain\"", FAIL_CHAR, IMAGE_CMD);
      return;
   }

   /*
    * Trigger the exposure
    */
   if ((rc = ASIStartExposure(serv_info->asi_camera_info->CameraID, 
			      ASI_FALSE)) != ASI_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) Unable to start exposure: rc=%d",
		__FILE__, __LINE__, rc);
      sprintf(buffer,
	      "%c %s \"Unable to start exposure\"", FAIL_CHAR, IMAGE_CMD);
      return;
   }
   
   /*
    * Wait for the image to be ready
    */
   ASIGetExpStatus(serv_info->asi_camera_info->CameraID, &asi_exp_status);
   while (asi_exp_status == ASI_EXP_WORKING) {
      usleep(5000);
      ASIGetExpStatus(serv_info->asi_camera_info->CameraID, &asi_exp_status);
   }
   if (asi_exp_status != ASI_EXP_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) Exposure request failed: rc=%d",
		__FILE__, __LINE__, rc);
      sprintf(buffer,
	      "%c %s \"Exposure request failed\"", FAIL_CHAR, IMAGE_CMD);
      return;
   }

   /*
    * Allocate memory for the image, if necessary
    */
   size = serv_info->asi_camera_info->MaxWidth * 
      serv_info->asi_camera_info->MaxHeight * sizeof(short);
   if (serv_info->image_data == NULL) {
      serv_info->image_data = 
	 (unsigned char *)cli_malloc(size);
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) Allocate memory for image (size=%d bytes)",
		__FILE__, __LINE__, size);
   }
   
   /*
    * Read out the image
    */
   if ((rc = ASIGetDataAfterExp(serv_info->asi_camera_info->CameraID, 
				serv_info->image_data, size)) != ASI_SUCCESS) {
      cfht_logv(CFHT_MAIN, CFHT_LOGONLY,
		"(%s:%d) Unable to read out image: rc=%d",
		__FILE__, __LINE__, rc);
      sprintf(buffer,
	      "%c %s \"Unable to read out image\"", FAIL_CHAR, IMAGE_CMD);
      return;
   }

   /*
    * Set the time for when the exposure was completed
    */
   serv_info->exp_readout_done_ts = getClockTime();
   serv_info->exp_cycle_time 
      = serv_info->exp_readout_done_ts - serv_info->exp_start_ts;
   time(&(serv_info->last_exp_completion));

   /*
    * Create a FITS image from the pixel data and send it to the file
    */
   if (writeFITSImage((unsigned short *)(serv_info->image_data), fd) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) unable to create FITS file", __FILE__, __LINE__);
      sprintf(buffer, "%c %s \"Unable to save temporary file on the guide"
	      " server\"", FAIL_CHAR, IMAGE_CMD);
      cfht_logv(CFHT_MAIN, CFHT_DEBUG,
		"(%s:%d) SEND> %s", __FILE__, __LINE__, buffer);
      close(fd);
      return;
   }
   
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
   cinfo->width = serv_info->image_width;
   cinfo->height = serv_info->image_height;

   /*
    * If we made it this far, send back a response with the number of bytes
    * of binary data that can be expected to be received from the server.
    */
   sprintf(buffer, "%c %ld", PASS_CHAR, file_stat.st_size);
   close(fd);

   return;
}


static PASSFAIL
com_exit(const char* arg)
{
   serv_info->response_buffer[0] = '\0'; /* Disconnect the client */
   return PASS;
}


static Command comlist[] = {
   { "etime <sec>",	 com_etime,	 "Set exposure time; <sec> can be a floating point number" },
   { "gain <0..510>",    com_gain,       "Set camera gain [0..510]" },
   { "exit",		 com_exit,	 "Exit connection" },
   { "quit",		 com_exit,	 "(Synonym for exit)" },
   { "bye",		 com_exit,	 "(Synonym for exit)" },
   { "logout",		 com_exit,	 "(Synonym for exit)" },
   { 0, 0, 0 }
};


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


static void
client_del(void* cinfo, char* buffer)
{
   if ((((client_info_t *)cinfo)->hostname) != NULL) {
      free(((client_info_t *)cinfo)->hostname);
   }
   free(cinfo);
}

static void
client_recv(void *cinfo, char* buffer)
{
   serv_info->response_buffer = buffer;

   /*
    * If this is an "image" request handle this in a special way.  Any time
    * images are requested, any video currently in progress must be stopped.
    */
   if (stristr(buffer, IMAGE_CMD) != NULL) {
      takeImage((client_info_t *)cinfo, buffer);

      return;
   }

   /*
    * Look up "buffer" as a command in the command table.
    * cli_execute is intended for "agents", so it may display
    * some messages on the SERVER's stdout or stderr.
    */
   if (cli_execute(buffer) != PASS) {
      strcpy(buffer, "! Error");
   }
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
    * Cleanup the listening socket 
    */
   if (serv_info->zwo_serv != NULL) {
      sockserv_destroy(serv_info->zwo_serv);
      serv_info->zwo_serv = NULL;
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
   setenv("CFHTDEBUG", "On", 1);

   /*
    * Initialize the CFHT logging stuff.
    */
   cfht_log(CFHT_MAIN, CFHT_LOG_ID, argv[0]);
   cfht_logv(CFHT_MAIN, CFHT_START, "%s", argv[0]);

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
   serv_info->response_buffer = (char *)cli_malloc(256);
   
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

   if (cli_init("zwoserv", comlist, 0) == FAIL) {
      exit(EXIT_FAILURE);
   }

   /*
    * Handle termination and interrupt signals to close the shutter and exit
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
		" the ZWO camera", __FILE__, __LINE__);
      exit(EXIT_FAILURE);
   }

   /* 
    * Set up the server parameters
    */
   serv_info->zwo_serv = sockserv_create(ZWOSERV_PORT);
   if (!(serv_info->zwo_serv)) {
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
   if (ssPutString(SS_PORT, ZWOSERV_PORT) != PASS) {
      cfht_logv(CFHT_MAIN, CFHT_ERROR,
		"(%s:%d) ssPutString on %s with %s failed: %s",
		__FILE__, __LINE__, SS_PORT, ZWOSERV_PORT, ssGetStrError());
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
   serv_info->zwo_serv->client_add_hook = client_add;
   serv_info->zwo_serv->client_del_hook = client_del;
   serv_info->zwo_serv->client_recv_hook = client_recv;
   serv_info->zwo_serv->client_send_binary_hook = client_send_binary;

   cfht_logv(CFHT_MAIN, CFHT_LOGONLY, 
	     "(%s:%d) Camera is ready to answer requests",
	     __FILE__, __LINE__);

   /* 
    * Go through a loop processing any commands sent by the client.
    */
   for (;;) {

      cli_signal_block(SIGTERM);
      cli_signal_block(SIGINT);

      sockserv_run(serv_info->zwo_serv, SOCKSERV_IDLE_POLL_INTERVAL);

      cli_signal_unblock(SIGTERM);
      cli_signal_unblock(SIGINT);
   }
   exit(EXIT_SUCCESS);
}
