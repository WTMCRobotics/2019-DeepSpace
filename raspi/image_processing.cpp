#define VISION_HEADER __builtin_bswap32(0x6101FEED)
#define VISION_INFO __builtin_bswap32(0x6101DA7A)
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdint>
#include <csignal>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
//#include <cameraserver/CameraServer.h>
#define PORT     3805
#define MAXLINE 1024
// #include <wiringSerial.h>
#pragma region
/*
 * wiringSerial.c:
 *	Handle a serial port
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

//#include <stdio.h>
//#include <stdlib.h>
//#include <stdint.h>
#include <stdarg.h>
//#include <string.h>
#include <termios.h>
//#include <unistd.h>
//#include <fcntl.h>
#include <sys/ioctl.h>
//#include <sys/types.h>
#include <sys/stat.h>

/*
 * serialOpen:
 *	Open and initialise the serial port, setting all the right
 *	port parameters - or as many as are required - hopefully!
 *********************************************************************************
 */

int serialOpen (const char *device, const int baud)
{
  struct termios options ;
  speed_t myBaud ;
  int     status, fd ;

  switch (baud)
  {
    case      50:	myBaud =      B50 ; break ;
    case      75:	myBaud =      B75 ; break ;
    case     110:	myBaud =     B110 ; break ;
    case     134:	myBaud =     B134 ; break ;
    case     150:	myBaud =     B150 ; break ;
    case     200:	myBaud =     B200 ; break ;
    case     300:	myBaud =     B300 ; break ;
    case     600:	myBaud =     B600 ; break ;
    case    1200:	myBaud =    B1200 ; break ;
    case    1800:	myBaud =    B1800 ; break ;
    case    2400:	myBaud =    B2400 ; break ;
    case    4800:	myBaud =    B4800 ; break ;
    case    9600:	myBaud =    B9600 ; break ;
    case   19200:	myBaud =   B19200 ; break ;
    case   38400:	myBaud =   B38400 ; break ;
    case   57600:	myBaud =   B57600 ; break ;
    case  115200:	myBaud =  B115200 ; break ;
    case  230400:	myBaud =  B230400 ; break ;
    case  460800:	myBaud =  B460800 ; break ;
    case  500000:	myBaud =  B500000 ; break ;
    case  576000:	myBaud =  B576000 ; break ;
    case  921600:	myBaud =  B921600 ; break ;
    case 1000000:	myBaud = B1000000 ; break ;
    case 1152000:	myBaud = B1152000 ; break ;
    case 1500000:	myBaud = B1500000 ; break ;
    case 2000000:	myBaud = B2000000 ; break ;
    case 2500000:	myBaud = B2500000 ; break ;
    case 3000000:	myBaud = B3000000 ; break ;
    case 3500000:	myBaud = B3500000 ; break ;
    case 4000000:	myBaud = B4000000 ; break ;
    default:
      return -2 ;
  }

  if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    return -1 ;

  fcntl (fd, F_SETFL, O_RDWR) ;

// Get and modify current options:

  tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);

  usleep (10000) ;	// 10mS

  return fd ;
}


/*
 * serialFlush:
 *	Flush the serial buffers (both tx & rx)
 *********************************************************************************
 */

void serialFlush (const int fd)
{
  tcflush (fd, TCIOFLUSH) ;
}


/*
 * serialClose:
 *	Release the serial port
 *********************************************************************************
 */

void serialClose (const int fd)
{
  close (fd) ;
}


/*
 * serialPutchar:
 *	Send a single character to the serial port
 *********************************************************************************
 */

void serialPutchar (const int fd, const unsigned char c)
{
  write (fd, &c, 1) ;
}


/*
 * serialPuts:
 *	Send a string to the serial port
 *********************************************************************************
 */

void serialPuts (const int fd, const char *s)
{
  write (fd, s, strlen (s)) ;
}

/*
 * serialPrintf:
 *	Printf over Serial
 *********************************************************************************
 */

void serialPrintf (const int fd, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  serialPuts (fd, buffer) ;
}


/*
 * serialDataAvail:
 *	Return the number of bytes of data avalable to be read in the serial port
 *********************************************************************************
 */

int serialDataAvail (const int fd)
{
  int result ;

  if (ioctl (fd, FIONREAD, &result) == -1)
    return -1 ;

  return result ;
}


/*
 * serialGetchar:
 *	Get a single character from the serial device.
 *	Note: Zero is a valid character and this function will time-out after
 *	10 seconds.
 *********************************************************************************
 */

int serialGetchar (const int fd)
{
  uint8_t x ;

  if (read (fd, &x, 1) != 1)
    return -1 ;

  return ((int)x) & 0xFF ;
}
#pragma endregion
#include <chrono>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std::chrono;

//#define JETSON // define if on jetson, undefine if on raspi
//#define CUDA // define to use GPU processing
#define SILENT // define to disable time checking
//#define TFMINI // define if using TFmini LiDAR (not working)

#ifdef CUDA
#include <opencv2/gpu/gpu.hpp>
#endif

/* 
serial frame:
| B | Contents |
| 4 | Header   |
| 4 | Angle    |
| 2 | Line X   |
| 2 | Line Y   |
| 2 | Distance |
| 2 | Checksum |

info frame:
| B | Contents |
| 4 | Header   |
| 4 | Threshold|
| 6 | Null     |
| 2 | Checksum |
 */

typedef struct {
    uint32_t header;
    float angle;
    int16_t lineX;
    int16_t lineY;
    uint16_t distance;
    uint16_t checksum;
} vision_frame_t;

uchar pixel_threshold = 160;
double brightness = 0.25;
bool verbose = true;
steady_clock::time_point t;
std::vector<int> times;
int serout;
//const char empty_frame[16] = {0x61, 0x01, 0xFE, 0xED, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xA0, 0x10};
vision_frame_t empty_frame = {VISION_HEADER, 0.0, 0, 0, 0, 0x10A0};
uint8_t camera_mappings[4];
uint8_t current_camera = 0;
uint8_t max_camera = 0;

// Ultrasonic functions because I'm too lazy to rewrite it all from the console
extern void initTime();
extern double getTime();

// Driver code
int sockfd = 3;

bool initSocket() {
    /*int sockfd, n;

    struct sockaddr_in serv_addr;
    struct hostent *server;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) return false;
    server = gethostbyname("10.61.1.2");
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        return false;
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(3805);
    if (connect(sockfd,(struct sockaddr *)&serv_addr,sizeof(serv_addr)) < 0) {
        if (errno == ECONNRESET || errno == ECONNREFUSED || errno == EINVAL) {
            close(sockfd);
            return false;
        }
        return false;
    } else fprintf(stderr, "Connected.\n");*/
    return true;
}

ssize_t sendFrame(char * hello, unsigned int size) {
    /*int i;
    if (sockfd == 3) return 0;
    for (i = 0; i < size; i += 256) {
        int n = ::write(sockfd, &hello[i], (size - i < 256 ? size - i : 256));
        if (n < 0) {
            perror("Got error");
            close(sockfd);
            return n;
        } else if (n != (size - i < 256 ? size - i : 256)) return i + n;
        else fprintf(stderr, "Sent data\n");
    }
    return i;*/
    return fwrite(hello, 1, size, stdout);
}

//cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("Vision Camera", 640, 480);

void outputImage(Mat cdst, bool success) {
    //std::cerr << "Outputting...\n";
    //std::vector<uchar> buf;
    //imencode(".raw", cdst, buf);
    //std::cout << std::string((char*)&buf[0], buf.size());
    /*ssize_t sz = sendFrame((char*)cdst.data, cdst.dataend - cdst.datastart);
    if (sz != cdst.dataend - cdst.datastart) {
        if (sz == -1) {
            std::cerr << "Error code: " << errno << ": ";
            while (!initSocket()) ;
        }
        std::cerr << "Failed to send image! " << sz << "\n";
    }*/
    //outputStream.PutFrame(cdst);
    #ifndef SILENT
    if (success) {
        std::cerr << "Got frame in " << (duration_cast<milliseconds>(steady_clock::now() - t)).count() << "ms\n";
        times.push_back((duration_cast<milliseconds>(steady_clock::now() - t)).count());
    }
    #endif
    if (!success) {
        std::cerr << "Failed to get frame\n";
        ::write(serout, &empty_frame, 16);
    }
}

void getResults(int signal) {
    int avg = 0;
    for (int i : times) avg += i;
    avg /= times.size();
    std::cerr << "Average time = " << avg << " ms\n";
    exit(0);
}

int getRectCount(VideoCapture capture) {
    Mat frame, cdst(Size(640, 480*2), CV_8UC3), edges;
    #ifdef CUDA
    cv::gpu::GpuMat gpuframe, bw, gpuedges;
    #else
    Mat bw;
    #endif
    capture >> frame;
    if(frame.empty()) {
        std::cerr << "Failed to capture an image.\n";
        return 0;
    }
    if (verbose) frame.copyTo(cdst(Rect(0, 480, 640, 480)));
    // Get min/max values
    double min = 0, max = 0;
    #ifdef CUDA
    gpuframe.upload(frame);
    cv::gpu::cvtColor(gpuframe, bw, CV_BGR2GRAY);
    cv::gpu::minMaxLoc(bw, &min, &max);
    #else
    cvtColor(frame, bw, CV_BGR2GRAY);
    minMaxLoc(bw, &min, &max);
    #endif
    //std::cerr << "min = " << (int)min << ", max = " << (int)max << "\n";
    uchar thresh = ((max - min) * (pixel_threshold / 256.0)) + min;
    // Convert to B/W image
    #ifdef CUDA
    cv::gpu::threshold(bw, gpuedges, (double)thresh, 255.0, THRESH_BINARY);
    gpuedges.download(edges);
    #else
    threshold(bw, edges, (double)thresh, 255.0, THRESH_BINARY);
    #endif
    if (verbose) {
        Mat edgesrgb;
        cvtColor(edges, edgesrgb, CV_GRAY2BGR);
        edgesrgb.copyTo(cdst(Rect(0, 0, 640, 480)));
    }
    // extract contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    return contours.size();
}

void frameChecksum(vision_frame_t &frame) {
    uint16_t * frame16 = (uint16_t*)&frame;
    uint32_t sum = 0;
    for (int i = 0; i < 7; i++) sum += frame16[i];
    frame.checksum = ~((uint16_t)(sum & 0xFFFF) + (uint16_t)(sum >> 16));
}

void calibrateVision(VideoCapture capture) {
    std::cerr << "Calibrating...\n";
    for (int c = getRectCount(capture); c != 1; c = getRectCount(capture)) {
        if (c == 0) pixel_threshold++;
        else pixel_threshold--;
    }
    std::cerr << "Done.\n";
    vision_frame_t frames;
    frames.header = VISION_INFO;
    frames.angle = (float)pixel_threshold;
    frameChecksum(frames);
    ::write(serout, &frames, 16);
    std::ofstream out("~/threshold.txt");
    if (out.is_open()) {
        out.put(pixel_threshold);
        out.close();
    }
}

int begin(int argc, const char * argv[]) {
    // Set up capture
    if (argc < 2) {
        std::cerr << "Missing camera IDs, please add the IDs to the input.\n";
    } else {
        std::string base_ids(argv[1]);
        max_camera = (base_ids.size() / 2) - 1;
        for (int i = 0; i < base_ids.size(); i+=2) {
std::cerr << base_ids.substr(i, 1) << "\n";
camera_mappings[i/2] = std::stoi(base_ids.substr(i, 1));
}
    }
    #ifdef JETSON
    serout = serialOpen("/dev/ttyTHS2", 9600);
    #else
    serout = serialOpen("/dev/ttyS0", 9600);
    #endif
    #ifdef TFMINI
    int serin = serialOpen("/dev/ttyUSB0", 115200);
    #endif
    //if (argc > 1) verbose = true;
    initTime();
    if (!initSocket()) {
        std::cerr << "Failed to open socket.\n";
        return 7;
    }
    std::ifstream in("~/threshold.txt");
    if (in.is_open()) {
        pixel_threshold = in.get();
        in.close();
    }
    fcntl (serout, F_SETFL, O_NONBLOCK);
    {
        uint32_t frames[4];
        frames[0] = VISION_INFO;
        ((float*)(frames))[1] = (float)pixel_threshold;
        for (int i = 8; i < 14; i++) ((uint8_t*)(frames))[i] = 0;
        uint32_t sum = ((VISION_INFO >> 16) + (VISION_INFO & 0xFFFF) + ((uint16_t*)(frames))[2]) + ((uint16_t*)(frames))[3];
        ((uint16_t*)(frames))[7] = ~((uint16_t)(sum & 0xFFFF) + (uint16_t)(sum >> 16));
        ::write(serout, &frames, 16);
    }
    signal(SIGINT, getResults);
    signal(SIGTERM, getResults);
    signal(SIGHUP, getResults);
    signal(SIGKILL, getResults);
    signal(SIGPIPE, getResults);
}

void processFrame(Mat &frame) {
        t = steady_clock::now();
        vision_frame_t frames;
        bool success = false;
        frames.header = VISION_HEADER;
        Mat cdst(Size(640, 480), CV_8UC3), edges;
        #ifdef CUDA
        cv::gpu::GpuMat gpuframe, bw, gpuedges;
        #else
        Mat bw;
        #endif
        if (current_camera != 0) cameraCapture >> cdst;
        if(frame.empty()) {
            std::cerr << "Failed to capture an image.\n";
            continue;
        }
        {
        char c = 0;
        if (serialDataAvail(serout)) {
            c = serialGetchar(serout);
            if (c == '=' || c == '+') {
                frames.header = VISION_INFO;
                frames.angle = (float)++pixel_threshold;
                frameChecksum(frames);
                ::write(serout, &frames, 16);
                frames.header = VISION_HEADER;
            } else if (c == '-' || c == '_') {
                frames.header = VISION_INFO;
                frames.angle = (float)--pixel_threshold;
                frameChecksum(frames);
                ::write(serout, &frames, 16);
                frames.header = VISION_HEADER;
            } else if (c == 'c') {
                calibrateVision(capture);
            } else if (c == 'n') {
                current_camera++;
                if (current_camera > max_camera) current_camera = 0;
                else {
                    cameraCapture.release();
                    cameraCapture = VideoCapture(camera_mappings[current_camera]);
                    cameraCapture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
                    cameraCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
                }
            }
        }
        //if (verbose) frame.copyTo(cdst(Rect(0, 480, 640, 480)));
        // Get min/max values
        double min = 0, max = 0;
        #ifdef CUDA
        gpuframe.upload(frame);
        cv::gpu::cvtColor(gpuframe, bw, CV_BGR2GRAY);
        cv::gpu::minMaxLoc(bw, &min, &max);
        #else
        cvtColor(frame, bw, CV_BGR2GRAY);
        minMaxLoc(bw, &min, &max);
        #endif
        //std::cerr << "min = " << (int)min << ", max = " << (int)max << "\n";
        uchar thresh = ((max - min) * (pixel_threshold / 256.0)) + min;
        // Convert to B/W image
        #ifdef CUDA
        cv::gpu::threshold(bw, gpuedges, (double)thresh, 255.0, THRESH_BINARY);
        gpuedges.download(edges);
        #else
        threshold(bw, edges, (double)thresh, 255.0, THRESH_BINARY);
        #endif
        if (verbose && current_camera == 0) {
            Mat edgesrgb;
            cvtColor(edges, edgesrgb, CV_GRAY2BGR);
            edgesrgb.copyTo(cdst(Rect(0, 0, 640, 480)));
        }
	/*
        std::vector<bool> lastRow(640, 0);
        for (int x = 0; x < frame.rows; x++) {
            bool lastValue = false;
            for (int y = 0; y < frame.cols; y++) {
                pixel_t pix = mpx(frame.at<Vec3b>(x, y).val);
                uchar avg = (pix.r + pix.g + pix.b) / 3;
                bool v = avg >= thresh;
                edges.at<uchar>(x, y) = v * 255;
                cdst.at<Vec3b>(x, y) = Vec3b(v * 255, v * 255, v * 255);
                cdst.at<Vec3b>(x + 480, y) = Vec3b(pix.b, pix.g, pix.r);
                lastValue = avg >= thresh;
                lastRow[y] = lastValue;
            }
        }*/
        // extract contours
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        if (contours.size() < 1) goto ErrorLine;
        {
        int maxAreaRect = 0;
        double maxArea = 0.0;
        for (int i = 0; i < contours.size(); i++) {
            double c = contourArea(contours[i], false);
            if (c > maxArea) {
                maxArea = c;
                maxAreaRect = i;
            }
        }
        // fit bounding rectangle around contour
        cv::RotatedRect rotatedRect = cv::minAreaRect(contours[maxAreaRect]);

        // read points and angle
        cv::Point2f rect_points[4];
        rotatedRect.points( rect_points );

        float angle = rotatedRect.angle; // angle
        // choose the longer edge of the rotated rect to compute the angle
        cv::Point2f edge1 = cv::Vec2f(rect_points[1].x, rect_points[1].y) - cv::Vec2f(rect_points[0].x, rect_points[0].y);
        cv::Point2f edge2 = cv::Vec2f(rect_points[2].x, rect_points[2].y) - cv::Vec2f(rect_points[1].x, rect_points[1].y);

        cv::Point2f usedEdge = edge1;
        if(cv::norm(edge2) > cv::norm(edge1))
            usedEdge = edge2;

        cv::Point2f reference = cv::Vec2f(1,0); // horizontal edge

        angle = (180.0f/CV_PI * acos((reference.x*usedEdge.x + reference.y*usedEdge.y) / (cv::norm(reference) *cv::norm(usedEdge)))) - 90.0;
        if (angle == 90.0 || angle == 0.0-90.0 || angle == 0.0 || angle == 45.0 || angle == 0.0-45.0) goto ErrorLine;
        // read center of rotated rect
        cv::Point2f center = rotatedRect.center; // center

        // draw rotated rect
        if (verbose && current_camera == 0)
            for(unsigned int j=0; j<4; ++j)
                cv::line(cdst, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,255,0));

        if (isnan(angle)) goto ErrorLine;
        {
        frames.angle = angle;

        // Calculate offset
        float multiplierX = (center.x - 320.0) / sqrt(pow(usedEdge.x, 2) + pow(usedEdge.y, 2));
        float multiplierY = (center.y - 240.0) / sqrt(pow(usedEdge.x, 2) + pow(usedEdge.y, 2));
        // Game manual says the length of the tape is 2"/5.08 cm,
        // we can find the distance from the center using the multiplier
        float cmDistanceX = multiplierX * 5.08;
        float cmDistanceY = multiplierY * 5.08;
        float inchDistanceX = multiplierX * 2.0;
        float inchDistanceY = multiplierY * 2.0;
        frames.lineX = (int16_t)(center.x - 320);
        frames.lineY = (int16_t)(center.y - 240);
        //frames[3] = *(uint32_t*)(&inchDistanceY);
        //std::cerr << cmDistance << "\n";
        if (verbose && current_camera == 0) cv::line(cdst, Point2f(320.0, center.y), Point2f(center.x, center.y), cv::Scalar(0, 255, 0));
        success = true;
        }}}
ErrorLine:
        // get distance data
        frames.distance = (uint16_t)getTime();
        //if (frames.distance > 325) frames.distance = ;

        // get checksum
        frameChecksum(frames);

        // send data out
        if (::write(serout, &frames, 18) != 18) ::write(serout, &empty_frame, 18;

        // output final image
        outputImage(frame, success);
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include "cameraserver/CameraServer.h"

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
       "switched cameras": [
           {
               "name": <virtual camera name>
               "key": <network table key used for selection>
               // if NT value is a string, it's treated as a name
               // if NT value is a double, it's treated as an integer index
           }
       ]
   }
 */

static const char* configFile = "/boot/frc.json";

namespace {

unsigned int team;
bool server = false;

struct CameraConfig {
  std::string name;
  std::string path;
  wpi::json config;
  wpi::json streamConfig;
};

struct SwitchedCameraConfig {
  std::string name;
  std::string key;
};

std::vector<CameraConfig> cameraConfigs;
std::vector<SwitchedCameraConfig> switchedCameraConfigs;
std::vector<cs::VideoSource> cameras;

wpi::raw_ostream& ParseError() {
  return wpi::errs() << "config error in '" << configFile << "': ";
}

bool ReadCameraConfig(const wpi::json& config) {
  CameraConfig c;

  // name
  try {
    c.name = config.at("name").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read camera name: " << e.what() << '\n';
    return false;
  }

  // path
  try {
    c.path = config.at("path").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "camera '" << c.name
                 << "': could not read path: " << e.what() << '\n';
    return false;
  }

  // stream properties
  if (config.count("stream") != 0) c.streamConfig = config.at("stream");

  c.config = config;

  cameraConfigs.emplace_back(std::move(c));
  return true;
}

bool ReadSwitchedCameraConfig(const wpi::json& config) {
  SwitchedCameraConfig c;

  // name
  try {
    c.name = config.at("name").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read switched camera name: " << e.what() << '\n';
    return false;
  }

  // key
  try {
    c.key = config.at("key").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "switched camera '" << c.name
                 << "': could not read key: " << e.what() << '\n';
    return false;
  }

  switchedCameraConfigs.emplace_back(std::move(c));
  return true;
}

bool ReadConfig() {
  // open config file
  std::error_code ec;
  wpi::raw_fd_istream is(configFile, ec);
  if (ec) {
    wpi::errs() << "could not open '" << configFile << "': " << ec.message()
                << '\n';
    return false;
  }

  // parse file
  wpi::json j;
  try {
    j = wpi::json::parse(is);
  } catch (const wpi::json::parse_error& e) {
    ParseError() << "byte " << e.byte << ": " << e.what() << '\n';
    return false;
  }

  // top level must be an object
  if (!j.is_object()) {
    ParseError() << "must be JSON object\n";
    return false;
  }

  // team number
  try {
    team = j.at("team").get<unsigned int>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read team number: " << e.what() << '\n';
    return false;
  }

  // ntmode (optional)
  if (j.count("ntmode") != 0) {
    try {
      auto str = j.at("ntmode").get<std::string>();
      wpi::StringRef s(str);
      if (s.equals_lower("client")) {
        server = false;
      } else if (s.equals_lower("server")) {
        server = true;
      } else {
        ParseError() << "could not understand ntmode value '" << str << "'\n";
      }
    } catch (const wpi::json::exception& e) {
      ParseError() << "could not read ntmode: " << e.what() << '\n';
    }
  }

  // cameras
  try {
    for (auto&& camera : j.at("cameras")) {
      if (!ReadCameraConfig(camera)) return false;
    }
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read cameras: " << e.what() << '\n';
    return false;
  }

  // switched cameras (optional)
  if (j.count("switched cameras") != 0) {
    try {
      for (auto&& camera : j.at("switched cameras")) {
        if (!ReadSwitchedCameraConfig(camera)) return false;
      }
    } catch (const wpi::json::exception& e) {
      ParseError() << "could not read switched cameras: " << e.what() << '\n';
      return false;
    }
  }

  return true;
}

cs::UsbCamera StartCamera(const CameraConfig& config) {
  wpi::outs() << "Starting camera '" << config.name << "' on " << config.path
              << '\n';
  auto inst = frc::CameraServer::GetInstance();
  cs::UsbCamera camera{config.name, config.path};
  auto server = inst->StartAutomaticCapture(camera);

  camera.SetConfigJson(config.config);
  camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

  if (config.streamConfig.is_object())
    server.SetConfigJson(config.streamConfig);

  return camera;
}

cs::MjpegServer StartSwitchedCamera(const SwitchedCameraConfig& config) {
  wpi::outs() << "Starting switched camera '" << config.name << "' on "
              << config.key << '\n';
  auto server =
      frc::CameraServer::GetInstance()->AddSwitchedCamera(config.name);

  nt::NetworkTableInstance::GetDefault()
      .GetEntry(config.key)
      .AddListener(
          [server](const auto& event) mutable {
            if (event.value->IsDouble()) {
              int i = event.value->GetDouble();
              if (i >= 0 && i < cameras.size()) server.SetSource(cameras[i]);
            } else if (event.value->IsString()) {
              auto str = event.value->GetString();
              for (int i = 0; i < cameraConfigs.size(); ++i) {
                if (str == cameraConfigs[i].name) {
                  server.SetSource(cameras[i]);
                  break;
                }
              }
            }
          },
          NT_NOTIFY_IMMEDIATE | NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);

  return server;
}

// example pipeline
class MyPipeline : public frc::VisionPipeline {
 public:
  int val = 0;

  void Process(cv::Mat& mat) override {
      processFrame(mat);
    ++val;
  }
};
}  // namespace

int main(int argc, char* argv[]) {
  if (argc >= 2) configFile = argv[1];

  // read configuration
  if (!ReadConfig()) return EXIT_FAILURE;

  // start NetworkTables
  auto ntinst = nt::NetworkTableInstance::GetDefault();
  if (server) {
    wpi::outs() << "Setting up NetworkTables server\n";
    ntinst.StartServer();
  } else {
    wpi::outs() << "Setting up NetworkTables client for team " << team << '\n';
    ntinst.StartClientTeam(team);
  }

  ::begin(argc, argv);

  // start cameras
  for (const auto& config : cameraConfigs)
    cameras.emplace_back(StartCamera(config));

  // start switched cameras
  for (const auto& config : switchedCameraConfigs) StartSwitchedCamera(config);

  // start image processing on camera 0 if present
  if (cameras.size() >= 1) {
    std::thread([&] {
      frc::VisionRunner<MyPipeline> runner(cameras[0], new MyPipeline(),
                                           [&](MyPipeline &pipeline) {
        // do something with pipeline results
      });
      /* something like this for GRIP:
      frc::VisionRunner<MyPipeline> runner(cameras[0], new grip::GripPipeline(),
                                           [&](grip::GripPipeline& pipeline) {
        ...
      });
       */
      runner.RunForever();
    }).detach();
  }

  // loop forever
  for (;;) std::this_thread::sleep_for(std::chrono::seconds(10));
}

