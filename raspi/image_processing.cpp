#define VISION_HEADER __builtin_bswap32(0x6101FEED)
#define VISION_INFO __builtin_bswap32(0x6101DA7A)
#include <iostream>
#include <vector>
#include <cstdint>
#include <csignal>
#include <fcntl.h>
#include <cmath>
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

#include <stdio.h>
#include <stdlib.h>
//#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
//#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
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
| 4 | Line     |
| 2 | Distance |
| 2 | Checksum |

info frame:
| B | Contents |
| 4 | Header   |
| 4 | Threshold|
| 6 | Null     |
| 2 | Checksum |
 */

uchar pixel_threshold = 160;
double brightness = 0.25;
bool verbose = false;
steady_clock::time_point t;
std::vector<int> times;
int serout;
const char empty_frame[16] = {0x61, 0x01, 0xFE, 0xED, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xA0, 0x10};

// Ultrasonic functions because I'm too lazy to rewrite it all from the console
extern void initTime();
extern double getTime();

void outputImage(Mat cdst, bool success) {
    if (verbose) {
        std::vector<uchar> buf;
        imencode(".jpg", cdst, buf);
        std::cout << std::string((char*)&buf[0], buf.size());
    }
    #ifndef SILENT
    if (success) {
        std::cerr << "Got frame in " << (duration_cast<milliseconds>(steady_clock::now() - t)).count() << "ms\n";
        times.push_back((duration_cast<milliseconds>(steady_clock::now() - t)).count());
    }
    #endif
    if (!success) {
        std::cerr << "Failed to get frame\n";
        ::write(serout, empty_frame, 16);
    }
}

void getResults(int signal) {
    int avg = 0;
    for (int i : times) avg += i;
    avg /= times.size();
    std::cerr << "Average time = " << avg << " ms\n";
    exit(0);
}

int main(int argc, const char * argv[]) {
    // Set up capture
    VideoCapture capture(0);
    if(!capture.isOpened()) {
        std::cerr << "Failed to connect to the camera.\n";
        return 1;
    }
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    capture.set(CV_CAP_PROP_CONTRAST, 1.0);
    capture.set(CV_CAP_PROP_BRIGHTNESS, brightness);
    #ifdef JETSON
    serout = serialOpen("/dev/ttyTHS2", 9600);
    #else
    serout = serialOpen("/dev/ttyS0", 9600);
    #endif
    #ifdef TFMINI
    int serin = serialOpen("/dev/ttyUSB0", 115200);
    #endif
    if (argc > 1) verbose = true;
    initTime();
    fcntl (serout, F_SETFL, O_NONBLOCK);
    {
        uint32_t frames[4];
        frames[0] = VISION_INFO;
        ((float*)(frames))[1] = (float)pixel_threshold;
        for (int i = 8; i < 14; i++) ((uint8_t*)(frames))[i] = 0;
        uint32_t sum = ((VISION_INFO >> 16) + (VISION_INFO & 0xFFFF) + ((uint16_t*)(frames))[2]) + ((uint16_t*)(frames))[3];
        ((uint16_t*)(frames))[7] = ~((uint16_t)(sum & 0xFFFF) + (uint16_t)(sum >> 16));
        ::write(serout, frames, 16);
    }
    signal(SIGINT, getResults);
    signal(SIGTERM, getResults);
    signal(SIGHUP, getResults);
    signal(SIGKILL, getResults);
    signal(SIGPIPE, getResults);
    while (true) {
        t = steady_clock::now();
        uint32_t frames[4];
        bool success = false;
        frames[0] = VISION_HEADER;
        Mat frame, cdst(Size(640, 480*2), CV_8UC3), edges;
        #ifdef CUDA
        cv::gpu::GpuMat gpuframe, bw, gpuedges;
        #else
        Mat bw;
        #endif
        capture >> frame;
        if(frame.empty()) {
            std::cerr << "Failed to capture an image.\n";
            goto ErrorLine;
        }
        {
        char c = 0;
        if (serialDataAvail(serout)) {
            c = serialGetchar(serout);
            if (c == '=' || c == '+') {
                frames[0] = VISION_INFO;
                ((float*)(frames))[1] = (float)++pixel_threshold;
                for (int i = 8; i < 14; i++) ((uint8_t*)(frames))[i] = 0;
                uint32_t sum = ((VISION_INFO >> 16) + (VISION_INFO & 0xFFFF) + ((uint16_t*)(frames))[2]) + ((uint16_t*)(frames))[3];
                ((uint16_t*)(frames))[7] = ~((uint16_t)(sum & 0xFFFF) + (uint16_t)(sum >> 16));
                ::write(serout, frames, 16);
                frames[0] = VISION_HEADER;
            } else if (c == '-' || c == '_') {
                frames[0] = VISION_INFO;
                ((float*)(frames))[1] = (float)--pixel_threshold;
                for (int i = 8; i < 14; i++) ((uint8_t*)(frames))[i] = 0;
                uint32_t sum = ((VISION_INFO >> 16) + (VISION_INFO & 0xFFFF) + ((uint16_t*)(frames))[2]) + ((uint16_t*)(frames))[3];
                ((uint16_t*)(frames))[7] = ~((uint16_t)(sum & 0xFFFF) + (uint16_t)(sum >> 16));
                ::write(serout, frames, 16);
                frames[0] = VISION_HEADER;
            }
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
        // fit bounding rectangle around contour
        cv::RotatedRect rotatedRect = cv::minAreaRect(contours[0]);

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
        if (verbose)
            for(unsigned int j=0; j<4; ++j)
                cv::line(cdst, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,255,0));

        if (isnan(angle)) goto ErrorLine;
        {
        frames[1] = *(uint32_t*)(&angle);
        
        // Calculate offset
        float multiplier = (center.x - 320.0) / sqrt(pow(usedEdge.x, 2) + pow(usedEdge.y, 2));
        // Game manual says the length of the tape is 2"/5.08 cm,
        // we can find the distance from the center using the multiplier
        float cmDistance = multiplier * 5.08;
        float inchDistance = multiplier * 2.0;
        frames[2] = *(uint32_t*)(&cmDistance);
        //std::cerr << cmDistance << "\n";
        if (verbose) cv::line(cdst, Point2f(320.0, center.y), Point2f(center.x, center.y), cv::Scalar(0, 255, 0));
        success = true;
        }}}
ErrorLine:
        // get distance data
        uint16_t * frames16 = (uint16_t*)frames;
        int16_t * frames16s = (int16_t*)frames;
        frames16s[6] = (uint16_t)getTime();
        if (frames16s[6] > 325) frames16s[6] = -1;

        // get checksum
        uint32_t sum = frames16[0] + frames16[1] + frames16[2] + 
        frames16[3] + frames16[4] + frames16[5] + frames16[6];
        frames16[7] = ~((uint16_t)(sum & 0xFFFF) + (uint16_t)(sum >> 16));

        // send data out
        ::write(serout, frames, 16);

        // output final image
        outputImage(cdst, success);
    }
    return 0;
}
