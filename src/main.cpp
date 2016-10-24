#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
extern "C" {
    #include "bm_dispatcher.h"
}
#include "bm_aruco2.h"
//#include "bm_bt_datastream.h"

/****************************************/
/****************************************/

void usage(FILE* stream, const char* prg) {
   fprintf(stream, "Usage:\n");
   fprintf(stream, "   %s <-s SIZE> [-f FILE]... [STREAM]...\n", prg);
   fprintf(stream, "   %s scan\n", prg);
   fprintf(stream, "Data repeater on various types of connections.\n");
   fprintf(stream, "\nBlabbermouth has two operational modes: streaming and scanning.\n");
   fprintf(stream, "\n== STREAMING ==\n\n");
   fprintf(stream, "In streaming mode, BlabberMouth connects to each STREAM passed as command line\n");
   fprintf(stream, "parameter and/or in FILE. Every time a message is sent by one of the peers over\n");
   fprintf(stream, "a stream, BlabberMouth collects the data and sends it over the other streams.\n\n");
   fprintf(stream, "Each message managed by BlabberMouth must be exactly SIZE bytes long, so the\n");
   fprintf(stream, "-s option is required.\n\n");
   fprintf(stream, "The syntax for stream descriptors is: ID:TYPE:DATA, where ID is a unique\n");
   fprintf(stream, "identifier for the stream; TYPE is a case-sensitive string such as 'tcp', 'bt',\n");
   fprintf(stream, "or 'xbee'; and DATA is a colon-separated string of fields that specify how to\n");
   fprintf(stream, "connect to the stream.\n\n");
   fprintf(stream, "Supported stream descriptors:\n\n");
   fprintf(stream, "  ID:tcp:SERVER:PORT      A TCP connection to SERVER on PORT\n");
   fprintf(stream, "  ID:bt:rfcomm:CHANNEL    An RFComm Bluetooth connection on CHANNEL\n");
   fprintf(stream, "  ID:xbee:ADDRESS:PORT    An XBee connection to ADDRESS on PORT\n");
   fprintf(stream, "\nOptions:\n\n");
   fprintf(stream, "  -s SIZE | --size SIZE   The size (in bytes) of a message\n");
   fprintf(stream, "  -f FILE | --file FILE   A file containing one stream descriptor per line\n");
   fprintf(stream, "\n== SCANNING ==\n\n");
   fprintf(stream, "In scanning mode, Blabbermouth looks for Bluetooth devices to connect to and\n");
   fprintf(stream, "prints a list of available devices.\n");
   fprintf(stream, "\n");
}

/****************************************/
/****************************************/

int file_parse(const char* fn,
               bm_dispatcher_t d) {
   /* Open the file */
   FILE* fd= fopen(fn, "r");
   if(!fd) {
      fprintf(stderr,
              "Can't open file '%s': %s\n",
              fn,
              strerror(errno));
      return 0;
   }
   /* Go through its content */
   char* line = NULL;
   char* start;
   char* end;
   size_t linelen = 0;
   while(getline(&line, &linelen, fd) >= 0) {
      /* Trim leading whitespace */
      start = line;
      while(*start != '\0' && isspace(*start)) ++start;
      /* Make sure the line is not empty or a comment */
      if(*start != '\0' && *start != '#') {
         /* Trim trailing whitespace */
         end = start + strlen(start) - 1;
         while(end != start && isspace(*end)) --end;
         *(end+1) = '\0';
         if(!bm_dispatcher_stream_add(d, start)) {
            free(line);
            fclose(fd);
            return 0;
         }
      }
   }
   /* All done */
   free(line);
   fclose(fd);
   return 1;
}

void resetposes(pose2d *p2d, int len)
{
    for(int i=0;i<len;i++)
    {
        p2d[i].idr=i+1;p2d[i].x=0;p2d[i].y=0;p2d[i].theta=0;
    }
}

/****************************************/
/****************************************/

int main(int argc, char* argv[]) {
    Mat TheInputImage;
    int nTags = 0, nStreams = 0;
    pose2d p2d[10];
    resetposes(p2d, 10);
    pose2d *poseBlock = p2d;
    MarkerDetector MDetector;
    VideoCapture TheVideoCapturer;
    CameraParameters TheCameraParameters;

    // read camera parameters if passed
    TheCameraParameters.readFromXMLFile("camera.yml");
    if (TheCameraParameters.isValid())
        cout << "Got 3D camera calibration." << endl;
    float TheMarkerSize = 0.1;
    
    ///////////  OPEN VIDEO
    // read from camera or from  file
    int vIdx = 0, key =0;
    int waitTime = 1;
    try {
        cout << "Opening camera index " << vIdx << endl;
    TheVideoCapturer.open(vIdx);
    
    // check video is open
    if (!TheVideoCapturer.isOpened())  throw std::runtime_error("Could not open video");
    }
    catch(exception e){
        
    }
    
    ///// CONFIGURE DATA
    // read first image to get the dimensions
    TheVideoCapturer >> TheInputImage;
    if (TheCameraParameters.isValid())
        TheCameraParameters.resize(TheInputImage.size());
    
    MDetector.setDictionary(Dictionary::getTypeFromString("ARUCO"));//sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
    MDetector.setThresholdParams(7, 7);
    MDetector.setThresholdParamRange(2, 0);
    
   // Check whether arguments have been given
   if(argc < 2) {
      usage(stdout, argv[0]);
      return 1;
   }
   // Check the first argument to detect the mode
   if(strcmp(argv[1], "scan") == 0) {
      // Scanning mode
      if(argc > 2) {
         fprintf(stderr, "%s: mode 'scan' accepts no options\n", argv[0]);
         return -1;
      }
      // Execute bluetooth scan
//      if(!bm_bt_scan())
//         return EXIT_FAILURE;
   }
   else {
      // Streaming mode
      // Create the stream dispatcher
      bm_dispatcher_t d = bm_dispatcher_new(poseBlock);
      // Parse the arguments
      for(int i = 1; i < argc; ++i) {
         // Check options
         if(argv[i][0] == '-') {
            // Check whether argument starts with -f or --file
            if(strcmp(argv[i], "-f") == 0 ||
               strcmp(argv[i], "--file") == 0) {
               ++i;
               if(i >= argc) {
                  fprintf(stderr, "%s: expected file after -f and --file\n", argv[0]);
                  return EXIT_FAILURE;
               }
               fprintf(stdout, "Reading streams from %s\n", argv[i]);
               if(!file_parse(argv[i], d)) {
                  // Some error occurred
                  bm_dispatcher_destroy(d);
                  return EXIT_FAILURE;
               }
            }
            else if(strcmp(argv[i], "-s") == 0 ||
                    strcmp(argv[i], "--size") == 0) {
               ++i;
               if(i >= argc) {
                  fprintf(stderr, "%s: expected size value after -s and --size\n", argv[0]);
                  bm_dispatcher_destroy(d);
                  return EXIT_FAILURE;
               }
               char* endptr;
               d->msg_len = strtol(argv[i], &endptr, 10);
               if(endptr == argv[i] || *endptr != '\0') {
                  fprintf(stderr, "%s: can't parse '%s' as a number\n", argv[0], argv[i]);
                  bm_dispatcher_destroy(d);
                  return EXIT_FAILURE;
               }
            }
            else {
               fprintf(stderr, "%s: %s: unknown option\n", argv[0], argv[i]);
               bm_dispatcher_destroy(d);
               return EXIT_FAILURE;
            }
         }
         else {
             // Not an option, consider it a stream descriptor
             bm_dispatcher_stream_add(d, argv[i]);
	     nStreams+=1;
         }
      }
      // Make sure required information has been passed
      if(d->msg_len == 0) {
         fprintf(stderr, "%s: option -s SIZE is required, with a value for SIZE > 0\n", argv[0]);
         return EXIT_FAILURE;
      }
      // Parsing done, start the execution
      //bm_dispatcher_execute(d);
       // Set signal handlers
       signal(SIGTERM, sighandler);
       signal(SIGINT, sighandler);
       while(nTags<nStreams)
	    getposes(MDetector, TheVideoCapturer, TheCameraParameters, TheMarkerSize, poseBlock, &nTags, 0);
       // Start all threads
       pthread_mutex_lock(&d->startmutex);
       d->start = 1;
       pthread_mutex_unlock(&d->startmutex);
       pthread_cond_broadcast(&d->startcond);
       // Wait for done signal
       ////////// GO !!!!
       do {
           getposes(MDetector, TheVideoCapturer, TheCameraParameters, TheMarkerSize, poseBlock, &nTags, 0);
//           cout << "\r Khepera " << p2d[1].idr << " : " << p2d[1].x << " m, " << p2d[1].y << " m, " << p2d[1].theta << " rad" << endl;
           pthread_mutex_lock(&d->startmutex);
           if(getactivet() == 0) setdone(1);
           pthread_mutex_unlock(&d->startmutex);
           key = cv::waitKey(waitTime); // wait for key to be pressed
       } while (key != 27 && (TheVideoCapturer.grab() ) && !isdone());
       // Cancel all threads
       for(bm_datastream_t s = d->streams;
           s != NULL;
           s = s->next)
           pthread_cancel(s->thread);
       // Wait for all threads to be done
       for(bm_datastream_t s = d->streams;
           s != NULL;
           s = s->next)
           pthread_join(s->thread, NULL);

      // All done
      bm_dispatcher_destroy(d);
   }

   return 1;
}

/****************************************/
/****************************************/
