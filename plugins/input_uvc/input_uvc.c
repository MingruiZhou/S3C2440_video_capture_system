#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <linux/videodev.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <pthread.h>
#include <syslog.h>

#include "../../utils.h"
#include "../../video_web.h"
#include "v4l2uvc.h"
#include "huffman.h"
#include "jpeg_utils.h"
#include <linux/fb.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include <jpeglib.h>

#include "jinclude.h"
#include "jpeglib.h"
#include "jerror.h"


#include <sys/socket.h>
#include <arpa/inet.h>
#include <time.h>
#include <syslog.h>

#include <dirent.h>
#define TEN_K (10*1024)
#define DBG_PRINTF printf
#define FB_DEVICE_NAME "/dev/fb0"
#define INPUT_PLUGIN_NAME "UVC webcam grabber"
#define MAX_ARGUMENTS 32
int dd,prev_size;
float k;
int	frameSizePerPic,maxFrameSize;
static int can_sd_save=1;
int iTopLeftX;
int iTopLeftY;

extern void jpeg_mem_src_tj(j_decompress_ptr, unsigned char *, unsigned long);

typedef struct PixelDatas {
	int iWidth;   /* 宽度: 一行有多少个象素 */
	int iHeight;  /* 高度: 一列有多少个象素 */
	int iBpp;     /* 一个象素用多少位来表示 */
	int iLineBytes;  /* 一行数据有多少字节 */
	int iTotalBytes; /* 所有字节数 */ 
	unsigned char *aucPixelDatas;  /* 象素数据存储的地方 */
}T_PixelDatas, *PT_PixelDatas;


typedef struct VideoBuf {
    T_PixelDatas tPixelDatas;
    int iPixelFormat;
}T_VideoBuf, *PT_VideoBuf;


typedef enum {
	VMS_FREE = 0,
	VMS_USED_FOR_PREPARE,
	VMS_USED_FOR_CUR,	
}E_VideoMemState;

typedef enum {
	PS_BLANK = 0,
	PS_GENERATING,
	PS_GENERATED,	
}E_PicState;


typedef struct VideoMem {
	int iID;                        /* ID值,用于标识不同的页面 */
	int bDevFrameBuffer;            /* 1: 这个VideoMem是显示设备的显存; 0: 只是一个普通缓存 */
	E_VideoMemState eVideoMemState; /* 这个VideoMem的状态 */
	E_PicState ePicState;           /* VideoMem中内存里图片的状态 */
	T_PixelDatas tPixelDatas;       /* 内存: 用来存储图像 */
	struct VideoMem *ptNext;        /* 链表 */
}T_VideoMem, *PT_VideoMem;

typedef struct DispOpr {
	char *name;              /* 显示模块的名字 */
	int iXres;               /* X分辨率 */
	int iYres;               /* Y分辨率 */
	int iBpp;                /* 一个象素用多少位来表示 */
	int iLineWidth;          /* 一行数据占据多少字节 */
	unsigned char *pucDispMem;   /* 显存地址 */
	int (*DeviceInit)(void);     /* 设备初始化函数 */
	int (*ShowPixel)(int iPenX, int iPenY, unsigned int dwColor);    /* 把指定座标的象素设为某颜色 */
	int (*CleanScreen)(unsigned int dwBackColor);                    /* 清屏为某颜色 */
	int (*ShowPage)(PT_PixelDatas ptPixelDatas);                         /* 显示一页,数据源自ptVideoMem */
	struct DispOpr *ptNext;      /* 链表 */
}T_DispOpr, *PT_DispOpr;

static PT_DispOpr g_ptDispOprHead;
static PT_DispOpr g_ptDefaultDispOpr;
static PT_VideoMem g_ptVideoMemHead;
T_VideoBuf tConvertBuf;

int DisplayInit(void);
int FBInit(void);
int RegisterDispOpr(PT_DispOpr ptDispOpr);
static int FBDeviceInit(void);
static int FBShowPixel(int iX, int iY, unsigned int dwColor);
static int FBCleanScreen(unsigned int dwBackColor);
void SelectAndInitDefaultDispDev(char *name);
PT_DispOpr GetDispOpr(char *pcName);
int GetDispResolution(int *piXres, int *piYres, int *piBpp);
int GetVideoBufForDisplay(PT_VideoBuf ptFrameBuf);
int PicZoom(PT_PixelDatas ptOriginPic, PT_PixelDatas ptZoomPic);

static T_DispOpr g_tFBOpr = {
	.name        = "fb",
	.DeviceInit  = FBDeviceInit,
	.ShowPixel   = FBShowPixel,
	.CleanScreen = FBCleanScreen,
};

//hyt convert
typedef struct MyErrorMgr
{
	struct jpeg_error_mgr pub;
	jmp_buf setjmp_buffer;
}T_MyErrorMgr, *PT_MyErrorMgr;


static int isSupportMjpeg2Rgb(int iPixelFormatIn, int iPixelFormatOut);

static int Mjpeg2RgbConvert(unsigned char *inputbuf, PT_VideoBuf ptVideoBufOut,int size);

static void MyErrorExit(j_common_ptr ptCInfo);

static int CovertOneLine(int iWidth, int iSrcBpp, int iDstBpp, unsigned char *pudSrcDatas, unsigned char *pudDstDatas);
int PicMerge(int iX, int iY, PT_PixelDatas ptSmallPic, PT_PixelDatas ptBigPic);
/*
 * UVC resolutions mentioned at: (at least for some webcams)
 * http://www.quickcamteam.net/hcl/frame-format-matrix/
 */
static const struct {
  const char *string;
  const int width, height;
}resolutions[] = {
  { "QSIF", 160,  120  },
  { "QCIF", 176,  144  },
  { "CGA",  320,  200  },
  { "QVGA", 320,  240  },
  { "CIF",  352,  288  },
  { "VGA",  640,  480  },
  { "SVGA", 800,  600  },
  { "XGA",  1024, 768  },
  { "SXGA", 1280, 1024 }
};

/* private functions and variables to this plugin */
pthread_t cam,sd_thread,lcd;
pthread_mutex_t controls_mutex;
struct vdIn *videoIn;
PT_VideoBuf ptVideoBufCur;
T_VideoBuf tZoomBuf;
static globals *pglobal;
static int gquality = 80;
static unsigned int minimum_size = 0;
static int dynctrls = 1;
static int g_fd;
static struct fb_var_screeninfo g_tFBVar;
static struct fb_fix_screeninfo g_tFBFix;			
static unsigned char *g_pucFBMem;
static unsigned char *g_pucFBMemTmp;
static unsigned int g_dwScreenSize;
static unsigned int g_dwLineWidth;
static unsigned int g_dwPixelWidth;
T_VideoBuf tFrameBuf;
int iLcdWidth;
int iLcdHeigt;
int iLcdBpp;
int iPixelFormatOfDisp;

void *cam_thread( void *);
void *sd_save_thread( void *);
void *lcd_thread( void *);


void cam_cleanup(void *);
void help(void);

/*** plugin interface functions ***/
/******************************************************************************
Description.: This function initializes the plugin. It parses the commandline-
              parameter and stores the default and parsed values in the
              appropriate variables.
Input Value.: param contains among others the command-line string
Return Value: 0 if everything is fine
              1 if "--help" was triggered, in this case the calling programm
              should stop running and leave.
******************************************************************************/
int input_init(input_parameter *param) 
{
  char *argv[MAX_ARGUMENTS]={NULL}, *dev = "/dev/video0", *s;
  int argc=1, width=640, height=480, fps=5, format=V4L2_PIX_FMT_MJPEG, i;
  in_cmd_type led = IN_CMD_LED_AUTO;

  /* initialize the mutes variable */
  if( pthread_mutex_init(&controls_mutex, NULL) != 0 ) 
  {
    IPRINT("could not initialize mutex variable\n");
    exit(EXIT_FAILURE);
  }

  /* convert the single parameter-string to an array of strings */
  argv[0] = INPUT_PLUGIN_NAME;
  if ( param->parameter_string != NULL && strlen(param->parameter_string) != 0 ) {
    char *arg=NULL, *saveptr=NULL, *token=NULL;

    arg=(char *)strdup(param->parameter_string);

    if ( strchr(arg, ' ') != NULL ) {
      token=strtok_r(arg, " ", &saveptr);
      if ( token != NULL ) {
        argv[argc] = strdup(token);
        argc++;
        while ( (token=strtok_r(NULL, " ", &saveptr)) != NULL ) {
          argv[argc] = strdup(token);
          argc++;
          if (argc >= MAX_ARGUMENTS) {
            IPRINT("ERROR: too many arguments to input plugin\n");
            return 1;
          }
        }
      }
    }
  }

  /* show all parameters for DBG purposes */
  for (i=0; i<argc; i++) {
    DBG("argv[%d]=%s\n", i, argv[i]);
  }

  /* parse the parameters */
  reset_getopt();
  while(1) {
    int option_index = 0, c=0;
    static struct option long_options[] = \
    {
      {"h", no_argument, 0, 0},
      {"help", no_argument, 0, 0},
      {"d", required_argument, 0, 0},
      {"device", required_argument, 0, 0},
      {"r", required_argument, 0, 0},
      {"resolution", required_argument, 0, 0},
      {"f", required_argument, 0, 0},
      {"fps", required_argument, 0, 0},
      {"y", no_argument, 0, 0},
      {"yuv", no_argument, 0, 0},
      {"q", required_argument, 0, 0},
      {"quality", required_argument, 0, 0},
      {"m", required_argument, 0, 0},
      {"minimum_size", required_argument, 0, 0},
      {"n", no_argument, 0, 0},
      {"no_dynctrl", no_argument, 0, 0},
      {"l", required_argument, 0, 0},
      {"led", required_argument, 0, 0},
      {0, 0, 0, 0}
    };

    /* parsing all parameters according to the list above is sufficent */
    c = getopt_long_only(argc, argv, "", long_options, &option_index);

    /* no more options to parse */
    if (c == -1) break;

    /* unrecognized option */
    if (c == '?'){
      help();
      return 1;
    }

    /* dispatch the given options */
    switch (option_index) {
      /* h, help */
      case 0:
      case 1:
        DBG("case 0,1\n");
        help();
        return 1;
        break;

      /* d, device */
      case 2:
      case 3:
        DBG("case 2,3\n");
        dev = strdup(optarg);
        break;

      /* r, resolution */
      case 4:
      case 5:
        DBG("case 4,5\n");
        width = -1;
        height = -1;

        /* try to find the resolution in lookup table "resolutions" */
        for ( i=0; i < LENGTH_OF(resolutions); i++ ) {
          if ( strcmp(resolutions[i].string, optarg) == 0 ) {
            width  = resolutions[i].width;
            height = resolutions[i].height;
          }
        }
        /* done if width and height were set */
        if(width != -1 && height != -1)
          break;
        /* parse value as decimal value */
        width  = strtol(optarg, &s, 10);
        height = strtol(s+1, NULL, 10);
        break;

      /* f, fps */
      case 6:
      case 7:
        DBG("case 6,7\n");
        fps=atoi(optarg);
        break;

      /* y, yuv */
      case 8:
      case 9:
        DBG("case 8,9\n");
        format = V4L2_PIX_FMT_YUYV;
        break;

      /* q, quality */
      case 10:
      case 11:
        DBG("case 10,11\n");
        format = V4L2_PIX_FMT_YUYV;
        gquality = MIN(MAX(atoi(optarg), 0), 100);
        break;

      /* m, minimum_size */
      case 12:
      case 13:
        DBG("case 12,13\n");
        minimum_size = MAX(atoi(optarg), 0);
        break;

      /* n, no_dynctrl */
      case 14:
      case 15:
        DBG("case 14,15\n");
        dynctrls = 0;
        break;

      /* l, led */
      case 16:
      case 17:
        DBG("case 16,17\n");
        if ( strcmp("on", optarg) == 0 ) {
          led = IN_CMD_LED_ON;
        } else if ( strcmp("off", optarg) == 0 ) {
          led = IN_CMD_LED_OFF;
        } else if ( strcmp("auto", optarg) == 0 ) {
          led = IN_CMD_LED_AUTO;
        } else if ( strcmp("blink", optarg) == 0 ) {
          led = IN_CMD_LED_BLINK;
        }
        break;

      default:
        DBG("default case\n");
        help();
        return 1;
    }
  }

  /* keep a pointer to the global variables */
  pglobal = param->global;
	/* 注册显示设备 hyt*/
	DisplayInit();
	SelectAndInitDefaultDispDev("fb");
	GetDispResolution(&iLcdWidth, &iLcdHeigt, &iLcdBpp);
    GetVideoBufForDisplay(&tFrameBuf);
    iPixelFormatOfDisp = tFrameBuf.iPixelFormat;
  /* allocate webcam datastructure */
  videoIn = malloc(sizeof(struct vdIn));
  if ( videoIn == NULL ) {
    IPRINT("not enough memory for videoIn\n");
    exit(EXIT_FAILURE);
  }
  memset(videoIn, 0, sizeof(struct vdIn));

  /* display the parsed values */
  IPRINT("Using V4L2 device.: %s\n", dev);
  IPRINT("Desired Resolution: %i x %i\n", width, height);
  IPRINT("Frames Per Second.: %i\n", fps);
  IPRINT("Format............: %s\n", (format==V4L2_PIX_FMT_YUYV)?"YUV":"MJPEG");
  if ( format == V4L2_PIX_FMT_YUYV )
    IPRINT("JPEG Quality......: %d\n", gquality);

  /* open video device and prepare data structure */
  if (init_videoIn(videoIn, dev, width, height, fps, format, 1) < 0) {
    IPRINT("init_VideoIn failed\n");
    closelog();
    exit(EXIT_FAILURE);
  }

  return 0;
}

/******************************************************************************
Description.: Stops the execution of worker thread
Input Value.: -
Return Value: always 0
******************************************************************************/
int input_stop(void) {
  DBG("will cancel input thread\n");
  pthread_cancel(cam);

  return 0;
}

/******************************************************************************
Description.: spins of a worker thread
Input Value.: -
Return Value: always 0
******************************************************************************/
int input_run(void) {
  pglobal->buf = malloc(videoIn->framesizeIn);
  if (pglobal->buf == NULL) 
  {
    fprintf(stderr, "could not allocate memory\n");
    exit(EXIT_FAILURE);
  }

 
  pthread_create(&cam, NULL, cam_thread, NULL);
  pthread_create(&lcd, NULL, lcd_thread, NULL);
  pthread_create(&sd_thread, NULL, cam_thread, NULL);  
  pthread_detach(cam);
  pthread_detach(lcd);
  pthread_detach(sd_thread);

  return 0;
}

/*** private functions for this plugin below ***/
/******************************************************************************
Description.: print a help message to stderr
Input Value.: -
Return Value: -
******************************************************************************/
void help(void) {
  int i;

  fprintf(stderr, " ---------------------------------------------------------------\n" \
                  " Help for input plugin..: "INPUT_PLUGIN_NAME"\n" \
                  " ---------------------------------------------------------------\n" \
                  " The following parameters can be passed to this plugin:\n\n" \
                  " [-d | --device ].......: video device to open (your camera)\n" \
                  " [-r | --resolution ]...: the resolution of the video device,\n" \
                  "                          can be one of the following strings:\n" \
                  "                          ");

  for ( i=0; i < LENGTH_OF(resolutions); i++ ) {
    fprintf(stderr, "%s ", resolutions[i].string);
    if ( (i+1)%6 == 0)
      fprintf(stderr, "\n                          ");
  }
  fprintf(stderr, "\n                          or a custom value like the following" \
                  "\n                          example: 640x480\n");

  fprintf(stderr, " [-f | --fps ]..........: frames per second\n" \
                  " [-y | --yuv ]..........: enable YUYV format and disable MJPEG mode\n" \
                  " [-q | --quality ]......: JPEG compression quality in percent \n" \
                  "                          (activates YUYV format, disables MJPEG)\n" \
                  " [-m | --minimum_size ].: drop frames smaller then this limit, useful\n" \
                  "                          if the webcam produces small-sized garbage frames\n" \
                  "                          may happen under low light conditions\n" \
                  " [-n | --no_dynctrl ]...: do not initalize dynctrls of Linux-UVC driver\n" \
                  " [-l | --led ]..........: switch the LED \"on\", \"off\", let it \"blink\" or leave\n" \
                  "                          it up to the driver using the value \"auto\"\n" \
                  " ---------------------------------------------------------------\n\n");
}

int check_for_filename_in(const struct dirent *entry) {
  int rc;

  int year, month, day, hour, minute, second;
  unsigned long long number;
  
  /*
   * try to scan the string using scanf
   * I would like to use a define for this format string later...
   */
  rc = sscanf(entry->d_name, "%d_%d_%d_%d_%d_%d_picture_%09llu.jpg", &year, \
                                                                     &month, \
                                                                     &day, \
                                                                     &hour, \
                                                                     &minute, \
                                                                     &second, \
                                                                     &number);

  DBG("%s, rc is %d (%d, %d, %d, %d, %d, %d, %llu)\n", entry->d_name, \
                                                       rc, \
                                                       year, \
                                                       month, \
                                                       day, \
                                                       hour, \
                                                       minute, \
                                                       second, \
                                                       number);

  /* if scanf could find all values, it matches our filenames */
  if (rc != 7) return 0;

  return 1;
}

void maintain_ringbuffer_in(int size) {
  struct dirent **namelist;
  int n, i;
  char buffer[1<<16];

  /* do nothing if ringbuffer is not set or wrong value is set */
  if (size < 0) return;

  /* get a sorted list of directory items */
  n = scandir("/sddisk/", &namelist, check_for_filename_in, alphasort);
  if (n < 0) {
    perror("scandir");
    return;
  }

  DBG("found %d directory entries\n", n);
 
  /* delete the first (thus oldest) number of files */
  for(i=0; i<(n-size); i++) {

    /* put together the folder name and the directory item */
    snprintf(buffer, sizeof(buffer), "%s/%s", "/sddisk/", namelist[i]->d_name);

    DBG("delete: %s\n", buffer);

    /* mark item for deletion */
    if ( unlink(buffer) == -1 ) {
      perror("could not delete file");
    }
   
    /* free allocated memory for name */
    free(namelist[i]);
  }

  /* keep the rest, but we still have to free every result */
  for(i=MAX(n-size, 0); i<n; i++) {
    DBG("keep: %s\n", namelist[i]->d_name);
    free(namelist[i]);
  }

  /* free last just allocated ressources */
  free(namelist);
}




/******************************************************************************
Description.: this thread worker grabs a frame and copies it to the global buffer
Input Value.: unused
Return Value: unused, always NULL
******************************************************************************/
void *cam_thread( void *arg ) {
	int iError,fd;	
	unsigned char *tmp_framebuffer=NULL;
	static unsigned char *frame=NULL;	
  /* set cleanup handler to cleanup allocated ressources */
  pthread_cleanup_push(cam_cleanup, NULL);
  while( !pglobal->stop ) 
  {

    /* grab a frame */
    if( uvcGrab(videoIn) < 0 ) {
      IPRINT("Error grabbing frames\n");
      exit(EXIT_FAILURE);
    }
  
    DBG("received frame of size: %d\n", videoIn->buf.bytesused);

    /*
     * Workaround for broken, corrupted frames:
     * Under low light conditions corrupted frames may get captured.
     * The good thing is such frames are quite small compared to the regular pictures.
     * For example a VGA (640x480) webcam picture is normally >= 8kByte large,
     * corrupted frames are smaller.
     */
    if ( videoIn->buf.bytesused < minimum_size ) {
      DBG("dropping too small frame, assuming it as broken\n");
      continue;
    }

    /* copy JPG picture to global buffer */
    pthread_mutex_lock( &pglobal->db );

    /*
     * If capturing in YUV mode convert to JPEG now.
     * This compression requires many CPU cycles, so try to avoid YUV format.
     * Getting JPEGs straight from the webcam, is one of the major advantages of
     * Linux-UVC compatible devices.
     */
    if (videoIn->formatIn == V4L2_PIX_FMT_YUYV) 
	{
      DBG("compressing frame\n");
      pglobal->size = compress_yuyv_to_jpeg(videoIn, pglobal->buf, videoIn->framesizeIn, gquality);
    }
    else 
	{
      DBG("copying frame\n");
      pglobal->size = memcpy_picture(pglobal->buf, videoIn->tmpbuffer, videoIn->buf.bytesused);
	  //start to restore picture
	  frameSizePerPic=pglobal->size;
	  
      pthread_cond_broadcast(&pglobal->db_update);
      pthread_mutex_unlock( &pglobal->db );
}

#if 0
//not very accurate
    /* motion detection can be done just by comparing the picture size, but it is not very accurate!! */
    if ( (prev_size - pglobal->size)*(prev_size - pglobal->size) > 4*1024*1024 ) 
	{
	    fd = open("/dev/led", 0);
		if (fd < 0) 
		{		
		    printf("open device leds");			
		}
		ioctl(fd, 1, 0);	
		close(fd);
        DBG("motion detected (delta: %d kB)\n", (prev_size - pglobal->size) / 1024);
    }
	else
	{		
	    fd = open("/dev/led", 0);
		if (fd < 0) 
		{		
		    printf("open device leds");			
		}
		ioctl(fd, 0, 0);	
		close(fd);
       // DBG("motion detected (delta: %d kB)\n", (prev_size - global->size) / 1024);
	}
    prev_size = pglobal->size;
#endif



    DBG("waiting for next frame\n");

    /* only use usleep if the fps is below 5, otherwise the overhead is too long */
    if ( videoIn->fps < 5 ) 
	{
      usleep(1000*1000/videoIn->fps);
    }
  }

  DBG("leaving input thread, calling cleanup function now\n");
  pthread_cleanup_pop(1);

  return NULL;
}

void *sd_save_thread( void *arg )
{
	char buffer1[1024] = {0}, buffer2[1024] = {0};
	int fd,frameSizePerPic=0,max_frame_size=0;
	int ringbufferExceed=0;//not allow exceed
	static unsigned char *frame=NULL,*tmp=NULL;	
	unsigned long long counter=0,ringbufferSize=100;//ringbufferSize ringbuffer size
	time_t t;
	struct tm *now;
	//pthread_cleanup_push(sd_save_thread, NULL);
	while( !pglobal->stop ) 
	{
	    /* wait for fresh frames */
   		pthread_cond_wait(&pglobal->db_update, &pglobal->db);

		    
			/* read buffer */
			frameSizePerPic = pglobal->size;

			if ( frameSizePerPic > max_frame_size )
			{	
			max_frame_size = frameSizePerPic+TEN_K;
			if ( (tmp = realloc(frame, max_frame_size)) == NULL ) 
			{
				free(frame);
				pthread_mutex_unlock( &pglobal->db );
				send_error(fd, 500, "not enough memory");
				return;
			}
				frame = tmp;
			}

			/* copy frame to our local buffer now */
			memcpy(frame, pglobal->buf, frameSizePerPic);   

			pthread_mutex_unlock( &pglobal->db );

			/* prepare filename *///output to sd card
			memset(buffer1, 0, sizeof(buffer1));
			memset(buffer2, 0, sizeof(buffer2));
			/* get current time */
			t = time(NULL);
			now = localtime(&t);
			if (now == NULL) 
			{
			perror("localtime");
			return NULL;
			}

			/* prepare string, add time and date values */
			if (strftime(buffer1, sizeof(buffer1), "%%s/%Y_%m_%d_%H_%M_%S_picture_%%09llu.jpg", now) == 0) 
			{
			OPRINT("strftime returned 0\n");
			free(frame); frame = NULL;
			return NULL;
			}

			snprintf(buffer2, sizeof(buffer2), buffer1, "/sddisk/", counter);

			counter++;
			DBG("writing file: %s\n", buffer2);

			/* open file for write */
			if( (fd = open(buffer2, O_CREAT|O_RDWR|O_TRUNC, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH)) < 0 ) 
			{
			OPRINT("could not open the file %s\n", buffer2);
			return NULL;
			}


			/* save picture to file */
			if( write(fd, frame, frameSizePerPic) < 0 ) 
			{
			OPRINT("could not write to file %s\n", buffer2);
			perror("write()");
			close(fd);
			return NULL;
			}
			close(fd);
			if ( ringbufferExceed <= 0 ) 
			{
			/* keep ringbuffer excactly at specified size */
			maintain_ringbuffer_in(ringbufferSize);
			} 		

	}
//	pthread_cleanup_pop(1);

}

void *lcd_thread( void *arg ) 
{
    int fd,frameSizePerPic=0,maxFrameSize=0,iError;
	static unsigned char *frame=NULL,*tmp_framebuffer=NULL;		
	while( !pglobal->stop ) 
	{	
	    pthread_cond_wait(&pglobal->db_update, &pglobal->db);

		
		iError=Mjpeg2RgbConvert(pglobal->buf,&tConvertBuf,pglobal->size);
        pthread_mutex_unlock( &pglobal->db );		
		ptVideoBufCur = &tConvertBuf;
		
		/* 确定缩放后的分辨率 */
		/* 把图片按比例缩放到VideoMem上, 居中显示
		 * 1. 先算出缩放后的大小
		 */
		
		k = (float)ptVideoBufCur->tPixelDatas.iHeight / ptVideoBufCur->tPixelDatas.iWidth;
		tZoomBuf.tPixelDatas.iWidth  = iLcdWidth;
		tZoomBuf.tPixelDatas.iHeight = iLcdWidth * k;
		if ( tZoomBuf.tPixelDatas.iHeight > iLcdHeigt)
		{
			tZoomBuf.tPixelDatas.iWidth  = iLcdHeigt / k;
			tZoomBuf.tPixelDatas.iHeight = iLcdHeigt;
		}
		tZoomBuf.tPixelDatas.iBpp		 = iLcdBpp;
		tZoomBuf.tPixelDatas.iLineBytes  = tZoomBuf.tPixelDatas.iWidth * tZoomBuf.tPixelDatas.iBpp / 8;
		tZoomBuf.tPixelDatas.iTotalBytes = tZoomBuf.tPixelDatas.iLineBytes * tZoomBuf.tPixelDatas.iHeight;
		
		if (!tZoomBuf.tPixelDatas.aucPixelDatas)
		{
			tZoomBuf.tPixelDatas.aucPixelDatas = malloc(tZoomBuf.tPixelDatas.iTotalBytes);
		}
		ptVideoBufCur = &tConvertBuf;
		PicZoom(&ptVideoBufCur->tPixelDatas, &tZoomBuf.tPixelDatas);
		ptVideoBufCur = &tZoomBuf;
		
		iTopLeftX = (iLcdWidth - ptVideoBufCur->tPixelDatas.iWidth) / 2;
		iTopLeftY = (iLcdHeigt - ptVideoBufCur->tPixelDatas.iHeight) / 2;
		PicMerge(iTopLeftX, iTopLeftY, &ptVideoBufCur->tPixelDatas, &tFrameBuf.tPixelDatas);
		memcpy(g_pucFBMem,	tFrameBuf.tPixelDatas.aucPixelDatas,g_dwScreenSize);
	}
}


/******************************************************************************
Description.: 
Input Value.: 
Return Value: 
******************************************************************************/
void cam_cleanup(void *arg) {
  static unsigned char first_run=1;

  if ( !first_run ) {
    DBG("already cleaned up ressources\n");
    return;
  }

  first_run = 0;
  IPRINT("cleaning up ressources allocated by input thread\n");

  close_v4l2(videoIn);
  if (videoIn->tmpbuffer != NULL) free(videoIn->tmpbuffer);
  if (videoIn != NULL) free(videoIn);
  if (pglobal->buf != NULL) free(pglobal->buf);
}




//hyt LCD相关代码
int DisplayInit(void)
{
	int iError;
	
	iError = FBInit();

	return iError;
}

int FBInit(void)
{
	return RegisterDispOpr(&g_tFBOpr);
}


int RegisterDispOpr(PT_DispOpr ptDispOpr)
{
	PT_DispOpr ptTmp;

	if (!g_ptDispOprHead)
	{
		g_ptDispOprHead   = ptDispOpr;
		ptDispOpr->ptNext = NULL;
	}
	else
	{
		ptTmp = g_ptDispOprHead;
		while (ptTmp->ptNext)
		{
			ptTmp = ptTmp->ptNext;
		}
		ptTmp->ptNext	  = ptDispOpr;
		ptDispOpr->ptNext = NULL;
	}

	return 0;
}

static int FBDeviceInit(void)
{
	int ret;
	//g_tFBVar.xres=480;
	//g_tFBVar.yres=272;
	g_fd = open(FB_DEVICE_NAME, O_RDWR);
	if (0 > g_fd)
	{
		DBG_PRINTF("can't open %s\n", FB_DEVICE_NAME);
	}

	ret = ioctl(g_fd, FBIOGET_VSCREENINFO, &g_tFBVar);
	//printf("xxx%d,YYY%d\r\n",g_tFBVar.xres, g_tFBVar.yres);
	if (ret < 0)
	{
		DBG_PRINTF("can't get fb's var\n");
		return -1;
	}

	ret = ioctl(g_fd, FBIOGET_FSCREENINFO, &g_tFBFix);
	if (ret < 0)
	{
		DBG_PRINTF("can't get fb's fix\n");
		return -1;
	}
	
	g_dwScreenSize = g_tFBVar.xres * g_tFBVar.yres * g_tFBVar.bits_per_pixel / 8;
	g_pucFBMem = (unsigned char *)mmap(NULL , g_dwScreenSize, PROT_READ | PROT_WRITE, MAP_SHARED, g_fd, 0);
	if (0 > g_pucFBMem)	
	{
		DBG_PRINTF("can't mmap\n");
		return -1;
	}
	g_tFBOpr.iXres       = g_tFBVar.xres;
	g_tFBOpr.iYres       = g_tFBVar.yres;
	g_tFBOpr.iBpp        = g_tFBVar.bits_per_pixel;
	g_tFBOpr.iLineWidth  = g_tFBVar.xres * g_tFBOpr.iBpp / 8;
	g_tFBOpr.pucDispMem  = g_pucFBMem;

	g_dwLineWidth  = g_tFBVar.xres * g_tFBVar.bits_per_pixel / 8;
	g_dwPixelWidth = g_tFBVar.bits_per_pixel / 8;
	
	return 0;
}


/**********************************************************************
 * 函数名称： FBShowPixel
 * 功能描述： 设置FrameBuffer的指定象素为某颜色
 * 输入参数： iX - 象素的X坐标
 *            iX - 象素的Y坐标
 *            dwColor - 颜色值,格式为32Bpp,即0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 0 - 成功, 其他值 - 失败
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2013/02/08	     V1.0	  韦东山	      创建
 ***********************************************************************/
static int FBShowPixel(int iX, int iY, unsigned int dwColor)
{
	unsigned char *pucFB;
	unsigned short *pwFB16bpp;
	unsigned int *pdwFB32bpp;
	unsigned short wColor16bpp; /* 565 */
	int iRed;
	int iGreen;
	int iBlue;

	if ((iX >= g_tFBVar.xres) || (iY >= g_tFBVar.yres))
	{
		return -1;
	}

	pucFB      = g_pucFBMem + g_dwLineWidth * iY + g_dwPixelWidth * iX;
	pwFB16bpp  = (unsigned short *)pucFB;
	pdwFB32bpp = (unsigned int *)pucFB;
	
	switch (g_tFBVar.bits_per_pixel)
	{
		case 8:
		{
			*pucFB = (unsigned char)dwColor;
			break;
		}
		case 16:
		{
			/* 从dwBackColor中取出红绿蓝三原色,
			 * 构造为16Bpp的颜色
			 */
			iRed   = (dwColor >> (16+3)) & 0x1f;
			iGreen = (dwColor >> (8+2)) & 0x3f;
			iBlue  = (dwColor >> 3) & 0x1f;
			wColor16bpp = (iRed << 11) | (iGreen << 5) | iBlue;
			*pwFB16bpp	= wColor16bpp;
			break;
		}
		case 32:
		{
			*pdwFB32bpp = dwColor;
			break;
		}
		default :
		{
			//DBG_PRINTF("can't support %d bpp\n", g_tFBVar.bits_per_pixel);
			return -1;
		}
	}

	return 0;
}

/**********************************************************************
 * 函数名称： FBCleanScreen
 * 功能描述： "framebuffer显示设备"的清屏函数
 * 输入参数： dwBackColor - 整个屏幕设置为该颜色
 * 输出参数： 无
 * 返 回 值： 0 - 成功, 其他值 - 失败
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2013/02/08	     V1.0	  韦东山	      创建
 ***********************************************************************/
static int FBCleanScreen(unsigned int dwBackColor)
{
	unsigned char *pucFB;
	unsigned short *pwFB16bpp;
	unsigned int *pdwFB32bpp;
	unsigned short wColor16bpp; /* 565 */
	int iRed;
	int iGreen;
	int iBlue;
	int i = 0;

	pucFB      = g_pucFBMem;
	pwFB16bpp  = (unsigned short *)pucFB;
	pdwFB32bpp = (unsigned int *)pucFB;

	switch (g_tFBVar.bits_per_pixel)
	{
		case 8:
		{
			memset(g_pucFBMem, dwBackColor, g_dwScreenSize);
			break;
		}
		case 16:
		{
			/* 从dwBackColor中取出红绿蓝三原色,
			 * 构造为16Bpp的颜色
			 */
			iRed   = (dwBackColor >> (16+3)) & 0x1f;
			iGreen = (dwBackColor >> (8+2)) & 0x3f;
			iBlue  = (dwBackColor >> 3) & 0x1f;
			wColor16bpp = (iRed << 11) | (iGreen << 5) | iBlue;
			while (i < g_dwScreenSize)
			{
				*pwFB16bpp	= wColor16bpp;
				pwFB16bpp++;
				i += 2;
			}
			break;
		}
		case 32:
		{
			while (i < g_dwScreenSize)
			{
				*pdwFB32bpp	= dwBackColor;
				pdwFB32bpp++;
				i += 4;
			}
			break;
		}
		default :
		{
			DBG_PRINTF("can't support %d bpp\n", g_tFBVar.bits_per_pixel);
			return -1;
		}
	}

	return 0;
}

//convert hyt
static int isSupportMjpeg2Rgb(int iPixelFormatIn, int iPixelFormatOut)
{
    if (iPixelFormatIn != V4L2_PIX_FMT_JPEG)
        return 0;
    if ((iPixelFormatOut != V4L2_PIX_FMT_RGB565) && (iPixelFormatOut != V4L2_PIX_FMT_RGB32))
    {
        return 0;
    }
    return 1;
}


METHODDEF(void)
init_mem_source (j_decompress_ptr cinfo)
{
  /* no work necessary here */
}

METHODDEF(boolean)
fill_mem_input_buffer (j_decompress_ptr cinfo)
{
  static JOCTET mybuffer[4];

  /* The whole JPEG data is expected to reside in the supplied memory
   * buffer, so any request for more data beyond the given buffer size
   * is treated as an error.
   */
  WARNMS(cinfo, JWRN_JPEG_EOF);
  /* Insert a fake EOI marker */
  mybuffer[0] = (JOCTET) 0xFF;
  mybuffer[1] = (JOCTET) JPEG_EOI;

  cinfo->src->next_input_byte = mybuffer;
  cinfo->src->bytes_in_buffer = 2;

  return TRUE;
}

METHODDEF(void)
skip_input_data (j_decompress_ptr cinfo, long num_bytes)
{
  struct jpeg_source_mgr * src = cinfo->src;

  /* Just a dumb implementation for now.  Could use fseek() except
   * it doesn't work on pipes.  Not clear that being smart is worth
   * any trouble anyway --- large skips are infrequent.
   */
  if (num_bytes > 0) {
    while (num_bytes > (long) src->bytes_in_buffer) {
      num_bytes -= (long) src->bytes_in_buffer;
      (void) (*src->fill_input_buffer) (cinfo);
      /* note we assume that fill_input_buffer will never return FALSE,
       * so suspension need not be handled.
       */
    }
    src->next_input_byte += (size_t) num_bytes;
    src->bytes_in_buffer -= (size_t) num_bytes;
  }
}

METHODDEF(void)
term_source (j_decompress_ptr cinfo)
{
  /* no work necessary here */
}





GLOBAL(void)
jpeg_mem_src_tj (j_decompress_ptr cinfo,
	      unsigned char * inbuffer, unsigned long insize)
{
  struct jpeg_source_mgr * src;

  if (inbuffer == NULL || insize == 0)	/* Treat empty input as fatal error */
    ERREXIT(cinfo, JERR_INPUT_EMPTY);

  /* The source object is made permanent so that a series of JPEG images
   * can be read from the same buffer by calling jpeg_mem_src only before
   * the first one.
   */
  if (cinfo->src == NULL) {	/* first time for this JPEG object? */
    cinfo->src = (struct jpeg_source_mgr *)
      (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
				  SIZEOF(struct jpeg_source_mgr));
  }

  src = cinfo->src;
  src->init_source = init_mem_source;
  src->fill_input_buffer = fill_mem_input_buffer;
  src->skip_input_data = skip_input_data;
  src->resync_to_restart = jpeg_resync_to_restart; /* use default method */
  src->term_source = term_source;
  src->bytes_in_buffer = (size_t) insize;
  src->next_input_byte = (JOCTET *) inbuffer;
}


static int Mjpeg2RgbConvert(unsigned char *inputbuf, PT_VideoBuf ptVideoBufOut,int size)
{
	struct jpeg_decompress_struct tDInfo;
	//struct jpeg_error_mgr tJErr;
    int iRet;
    int iRowStride;
    unsigned char *aucLineBuffer = NULL;
    unsigned char *pucDest;
	T_MyErrorMgr tJerr;
    PT_PixelDatas ptPixelDatas = &ptVideoBufOut->tPixelDatas;
	g_pucFBMemTmp=g_pucFBMem;

	// 分配和初始化一个decompression结构体
	//tDInfo.err = jpeg_std_error(&tJErr);

	tDInfo.err               = jpeg_std_error(&tJerr.pub);
	tJerr.pub.error_exit     = MyErrorExit;

	if(setjmp(tJerr.setjmp_buffer))
	{
		/* 如果程序能运行到这里, 表示JPEG解码出错 */
        jpeg_destroy_decompress(&tDInfo);
        if (aucLineBuffer)
        {
            free(aucLineBuffer);
        }
        if (ptPixelDatas->aucPixelDatas)
        {
            free(ptPixelDatas->aucPixelDatas);
        }
		return -1;
		printf("occer-error````````````\r\n");
	}

	jpeg_create_decompress(&tDInfo);

	// 用jpeg_read_header获得jpg信息
	//jpeg_stdio_src(&tDInfo, ptFileMap->tFp);
	/* 把数据设为内存中的数据 */
    jpeg_mem_src_tj (&tDInfo, inputbuf, size);
    //                        输入数据									数据长度										

    iRet = jpeg_read_header(&tDInfo, TRUE);

	// 设置解压参数,比如放大、缩小
    tDInfo.scale_num = 1;
	tDInfo.scale_denom = 1;
    
	// 启动解压：jpeg_start_decompress	
	jpeg_start_decompress(&tDInfo);
    
	// 一行的数据长度
	//printf("~~~~~tDInfo.output_width=%d,tDInfo.output_height=%d\r\n",tDInfo.output_width,tDInfo.output_height);
	iRowStride = tDInfo.output_width * tDInfo.output_components;
	aucLineBuffer = malloc(iRowStride);

    if (NULL == aucLineBuffer)
    {
   		 printf("~~~~~jpeg_read_headeraaaaa\r\n");
   	     return -1;
    }

	ptPixelDatas->iWidth  = tDInfo.output_width;
	ptPixelDatas->iHeight = tDInfo.output_height;
	ptPixelDatas->iBpp    = 16;
	ptPixelDatas->iLineBytes    = ptPixelDatas->iWidth * ptPixelDatas->iBpp / 8;
    ptPixelDatas->iTotalBytes   = ptPixelDatas->iHeight * ptPixelDatas->iLineBytes;
	if (NULL == ptPixelDatas->aucPixelDatas)
	{
	    ptPixelDatas->aucPixelDatas = malloc(ptPixelDatas->iTotalBytes);
	}

    pucDest = ptPixelDatas->aucPixelDatas;
//printf("tDInfo.output_height=%d,ptPixelDatas->iWidth=%d\r\n",tDInfo.output_height,ptPixelDatas->iWidth);
	// 循环调用jpeg_read_scanlines来一行一行地获得解压的数据
	while (tDInfo.output_scanline < tDInfo.output_height) 
	{
        /* 得到一行数据,里面的颜色格式为0xRR, 0xGG, 0xBB */
		(void) jpeg_read_scanlines(&tDInfo, &aucLineBuffer, 1);

		// 转到ptPixelDatas去
		CovertOneLine(ptPixelDatas->iWidth, 24, 16, aucLineBuffer, pucDest);
						//宽度           mjpeg都是24    目的bpp    
		pucDest += ptPixelDatas->iLineBytes;
	}	
	ptVideoBufOut->tPixelDatas.aucPixelDatas=ptPixelDatas->aucPixelDatas;

	free(aucLineBuffer);
	jpeg_finish_decompress(&tDInfo);
	jpeg_destroy_decompress(&tDInfo);

    return 0;
}


static void MyErrorExit(j_common_ptr ptCInfo)
{
    static char errStr[JMSG_LENGTH_MAX];
    
	PT_MyErrorMgr ptMyErr = (PT_MyErrorMgr)ptCInfo->err;

    /* Create the message */
    (*ptCInfo->err->format_message) (ptCInfo, errStr);
    DBG_PRINTF("%s\n", errStr);

	longjmp(ptMyErr->setjmp_buffer, 1);
}

static int CovertOneLine(int iWidth, int iSrcBpp, int iDstBpp, unsigned char *pudSrcDatas, unsigned char *pudDstDatas)
{
	unsigned int dwRed;
	unsigned int dwGreen;
	unsigned int dwBlue;
	unsigned int dwColor;

	unsigned short *pwDstDatas16bpp = (unsigned short *)pudDstDatas;
	unsigned int   *pwDstDatas32bpp = (unsigned int *)pudDstDatas;

	int i;
	int pos = 0;

	if (iSrcBpp != 24)
	{
		return -1;
	}

	if (iDstBpp == 24)
	{
		memcpy(pudDstDatas, pudSrcDatas, iWidth*3);
	}
	else
	{
		for (i = 0; i < iWidth; i++)
		{
			dwRed   = pudSrcDatas[pos++];
			dwGreen = pudSrcDatas[pos++];
			dwBlue  = pudSrcDatas[pos++];
			if (iDstBpp == 32)
			{
				dwColor = (dwRed << 16) | (dwGreen << 8) | dwBlue;
				*pwDstDatas32bpp = dwColor;
				pwDstDatas32bpp++;
			}
			else if (iDstBpp == 16)
			{
				/* 565 */
				dwRed   = dwRed >> 3;
				dwGreen = dwGreen >> 2;
				dwBlue  = dwBlue >> 3;
				dwColor = (dwRed << 11) | (dwGreen << 5) | (dwBlue);
				*pwDstDatas16bpp = dwColor;
				pwDstDatas16bpp++;
			}
		}
	}
	return 0;
}

int PicMerge(int iX, int iY, PT_PixelDatas ptSmallPic, PT_PixelDatas ptBigPic)
{
	int i;
	unsigned char *pucSrc;
	unsigned char *pucDst;
	
	if ((ptSmallPic->iWidth > ptBigPic->iWidth)  ||
		(ptSmallPic->iHeight > ptBigPic->iHeight) ||
		(ptSmallPic->iBpp != ptBigPic->iBpp))
	{
		return -1;
	}

	pucSrc = ptSmallPic->aucPixelDatas;
	pucDst = ptBigPic->aucPixelDatas + iY * ptBigPic->iLineBytes + iX * ptBigPic->iBpp / 8;
	for (i = 0; i < ptSmallPic->iHeight; i++)
	{
		memcpy(pucDst, pucSrc, ptSmallPic->iLineBytes);
		pucSrc += ptSmallPic->iLineBytes;
		pucDst += ptBigPic->iLineBytes;
	}
	return 0;
}



void SelectAndInitDefaultDispDev(char *name)
{
	g_ptDefaultDispOpr = GetDispOpr(name);
	if (g_ptDefaultDispOpr)
	{
		g_ptDefaultDispOpr->DeviceInit();
		g_ptDefaultDispOpr->CleanScreen(0);
	}
}

PT_DispOpr GetDispOpr(char *pcName)
{
	PT_DispOpr ptTmp = g_ptDispOprHead;
	
	while (ptTmp)
	{
		if (strcmp(ptTmp->name, pcName) == 0)
		{
			return ptTmp;
		}
		ptTmp = ptTmp->ptNext;
	}
	return NULL;
}

int GetDispResolution(int *piXres, int *piYres, int *piBpp)
{
	if (g_ptDefaultDispOpr)
	{
		*piXres = g_ptDefaultDispOpr->iXres;
		*piYres = g_ptDefaultDispOpr->iYres;
		*piBpp  = g_ptDefaultDispOpr->iBpp;
		return 0;
	}
	else
	{
		return -1;
	}
}

int GetVideoBufForDisplay(PT_VideoBuf ptFrameBuf)
{
    ptFrameBuf->iPixelFormat = (g_ptDefaultDispOpr->iBpp == 16) ? V4L2_PIX_FMT_RGB565 : \
                                   (g_ptDefaultDispOpr->iBpp == 32) ?  V4L2_PIX_FMT_RGB32 : \
                                           0;
    ptFrameBuf->tPixelDatas.iWidth  = g_ptDefaultDispOpr->iXres;
    ptFrameBuf->tPixelDatas.iHeight = g_ptDefaultDispOpr->iYres;
    ptFrameBuf->tPixelDatas.iBpp    = g_ptDefaultDispOpr->iBpp;
    ptFrameBuf->tPixelDatas.iLineBytes    = g_ptDefaultDispOpr->iLineWidth;
    ptFrameBuf->tPixelDatas.iTotalBytes   = ptFrameBuf->tPixelDatas.iLineBytes * ptFrameBuf->tPixelDatas.iHeight;
    ptFrameBuf->tPixelDatas.aucPixelDatas = g_ptDefaultDispOpr->pucDispMem;
    return 0;
}

int PicZoom(PT_PixelDatas ptOriginPic, PT_PixelDatas ptZoomPic)
{
    unsigned long dwDstWidth = ptZoomPic->iWidth;
    unsigned long* pdwSrcXTable;
	unsigned long x;
	unsigned long y;
	unsigned long dwSrcY;
	unsigned char *pucDest;
	unsigned char *pucSrc;
	unsigned long dwPixelBytes = ptOriginPic->iBpp/8;

    //printf("src:\n");
    //printf("%d x %d, %d bpp, data: 0x%x\n", ptOriginPic->iWidth, ptOriginPic->iHeight, ptOriginPic->iBpp, (unsigned int)ptOriginPic->aucPixelDatas);

    //printf("dest:\n");
    //printf("%d x %d, %d bpp, data: 0x%x\n", ptZoomPic->iWidth, ptZoomPic->iHeight, ptZoomPic->iBpp, (unsigned int)ptZoomPic->aucPixelDatas);

	if (ptOriginPic->iBpp != ptZoomPic->iBpp)
	{
		return -1;
	}

    pdwSrcXTable = malloc(sizeof(unsigned long) * dwDstWidth);
    if (NULL == pdwSrcXTable)
    {
        DBG_PRINTF("malloc error!\n");
        return -1;
    }
    for (x = 0; x < dwDstWidth; x++)//生成表 pdwSrcXTable
    {
        pdwSrcXTable[x]=(x*ptOriginPic->iWidth/ptZoomPic->iWidth);
    }

    for (y = 0; y < ptZoomPic->iHeight; y++)
    {			
        dwSrcY = (y * ptOriginPic->iHeight / ptZoomPic->iHeight);

		pucDest = ptZoomPic->aucPixelDatas + y*ptZoomPic->iLineBytes;
		pucSrc  = ptOriginPic->aucPixelDatas + dwSrcY*ptOriginPic->iLineBytes;
		
        for (x = 0; x <dwDstWidth; x++)
        {
            /* 原图座标: pdwSrcXTable[x]，srcy
             * 缩放座标: x, y
			 */
			 memcpy(pucDest+x*dwPixelBytes, pucSrc+pdwSrcXTable[x]*dwPixelBytes, dwPixelBytes);
        }
    }

    free(pdwSrcXTable);
	return 0;
}








