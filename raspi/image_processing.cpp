#define VISION_HEADER __builtin_bswap32(0x6101FEED)
#define VISION_INFO __builtin_bswap32(0x6101DA7A)
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <csignal>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <wiringSerial.h>
#include <opencv2/opencv.hpp>
using namespace cv;

#pragma pack(push, 1)
typedef struct {
    uchar b;
    uchar g;
    uchar r;
} pixel_t;
typedef struct {
    int x;
    int y;
} point_t;
#pragma pack(pop)
#define mpx(v) *(pixel_t*)(v);
#define cti(x, y) (((uint32_t)x << 16) | y)

uchar pixel_threshold = 192-32;
double brightness = 0.25;

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
| 1 | Threshold|
| 9 | Null     |
| 2 | Checksum |
 */
bool verbose = false;

void outputImage(Mat cdst) {
    if (verbose) {
        std::vector<uchar> buf;
        imencode(".jpg", cdst, buf);
        std::cout << std::string((char*)&buf[0], buf.size());
    }
}

int main(int argc, const char * argv[]) {
    // Set up capture
    VideoCapture capture(0);
    VideoWriter output("output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 15, Size(640, 480*2));
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    capture.set(CV_CAP_PROP_CONTRAST, 1.0);
    capture.set(CV_CAP_PROP_BRIGHTNESS, brightness);
    if(!capture.isOpened()) {
        std::cerr << "Failed to connect to the camera.\n";
        return 1;
    }
    int serout = serialOpen("/dev/ttyS0", 9600);
    #ifdef TFMINI
    int serin = serialOpen("/dev/ttyUSB0", 115200);
    #endif
    if (argc > 1) verbose = true;
    fcntl (serout, F_SETFL, O_NONBLOCK);
    {
        uint32_t frames[4];
        frames[0] = VISION_INFO;
        ((uint8_t*)(frames))[4] = pixel_threshold;
        for (int i = 5; i < 14; i++) ((uint8_t*)(frames))[i] = 0;
        uint32_t sum = ((VISION_INFO >> 16) + (VISION_INFO & 0xFFFF) + ((uint16_t*)(frames))[2]);
        ((uint16_t*)(frames))[7] = ~((uint16_t)(sum & 0xFFFF) + (uint16_t)(sum >> 16));
        ::write(serout, frames, 16);
    }
    while (true) {
        uint32_t frames[4];
        frames[0] = VISION_HEADER;
        Mat frame, edges(Size(640, 480), CV_8UC1), denoised, cdst(Size(640, 480*2), CV_8UC3);
        capture >> frame;
        if(frame.empty()) {
            std::cerr << "Failed to capture an image.\n";
            return 2;
        }
        char c = 0;
        if (serialDataAvail(serout)) c = serialGetchar(serout);
        if (c == '=' || c == '+') {
            frames[0] = VISION_INFO;
            ((uint8_t*)(frames))[4] = ++pixel_threshold;
            for (int i = 5; i < 14; i++) ((uint8_t*)(frames))[i] = 0;
            uint32_t sum = ((VISION_INFO >> 16) + (VISION_INFO & 0xFFFF) + ((uint16_t*)(frames))[2]);
            ((uint16_t*)(frames))[7] = ~((uint16_t)(sum & 0xFFFF) + (uint16_t)(sum >> 16));
            ::write(serout, frames, 16);
            frames[0] = VISION_HEADER;
        } else if (c == '-' || c == '_') {
            frames[0] = VISION_INFO;
            ((uint8_t*)(frames))[4] = --pixel_threshold;
            for (int i = 5; i < 14; i++) ((uint8_t*)(frames))[i] = 0;
            uint32_t sum = ((VISION_INFO >> 16) + (VISION_INFO & 0xFFFF) + ((uint16_t*)(frames))[2]);
            ((uint16_t*)(frames))[7] = ~((uint16_t)(sum & 0xFFFF) + (uint16_t)(sum >> 16));
            ::write(serout, frames, 16);
            frames[0] = VISION_HEADER;
        }
        // Get min/max values
        uchar min = 255, max = 0;
        for (int x = 0; x < frame.rows; x++) {
            for (int y = 0; y < frame.cols; y++) {
                pixel_t pix = mpx(frame.at<Vec3b>(x, y).val);
                uchar avg = (pix.r + pix.g + pix.b) / 3;
                if (avg > max) max = avg;
                if (avg < min) min = avg;
            }
        }
        //std::cerr << "min = " << (int)min << ", max = " << (int)max << "\n";
        uchar thresh = ((max - min) * (pixel_threshold / 256.0)) + min;
        // Convert to B/W image
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
        }
        // extract contours
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        if (contours.size() < 1) {outputImage(cdst); continue;}
        //for(int i=0; i<1; ++i)
        int i = 0;
        // fit bounding rectangle around contour
        cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);

        // read points and angle
        cv::Point2f rect_points[4]; 
        rotatedRect.points( rect_points );

        float  angle = rotatedRect.angle; // angle
        // choose the longer edge of the rotated rect to compute the angle
        cv::Point2f edge1 = cv::Vec2f(rect_points[1].x, rect_points[1].y) - cv::Vec2f(rect_points[0].x, rect_points[0].y);
        cv::Point2f edge2 = cv::Vec2f(rect_points[2].x, rect_points[2].y) - cv::Vec2f(rect_points[1].x, rect_points[1].y);

        cv::Point2f usedEdge = edge1;
        if(cv::norm(edge2) > cv::norm(edge1))
            usedEdge = edge2;

        cv::Point2f reference = cv::Vec2f(1,0); // horizontal edge

        angle = 180.0f/CV_PI * acos((reference.x*usedEdge.x + reference.y*usedEdge.y) / (cv::norm(reference) *cv::norm(usedEdge)));
        angle -= 90.0;            
        if (angle == 90.0 || angle == 0.0-90.0 || angle == 0.0 || angle == 45.0 || angle == 0.0-45.0) continue;
        // read center of rotated rect
        cv::Point2f center = rotatedRect.center; // center

        // draw rotated rect
        for(unsigned int j=0; j<4; ++j)
            cv::line(cdst, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,255,0));

        if (isnan(angle)) {outputImage(cdst); continue;}
        frames[1] = *(uint32_t*)(&angle);
        
        // Calculate offset
        float distance = (int)center.x - 320.0;
        float edgeLength = sqrt(pow(usedEdge.x, 2) + pow(usedEdge.y, 2));
        float multiplier = distance / edgeLength;
        // Game manual says the length of the tape is 2"/5.08 cm,
        // we can find the distance from the center using the multiplier
        float cmDistance = multiplier * 5.08;
        float inchDistance = multiplier * 2.0;
        frames[2] = *(uint32_t*)(&cmDistance);
        //std::cerr << cmDistance << "\n";
        if (verbose) cv::line(cdst, Point2f(320.0, 240.0), center, cv::Scalar(0, 255, 0));

        // TODO: get distance data
        uint16_t * frames16 = (uint16_t*)frames;
        frames16[6] = 0;

        // get checksum
        uint32_t sum = frames16[0] + frames16[1] + frames16[2] + 
        frames16[3] + frames16[4] + frames16[5] + frames16[6];
        frames16[7] = ~((uint16_t)(sum & 0xFFFF) + (uint16_t)(sum >> 16));

        // send data out
        ::write(serout, frames, 16);

        // output final image
        outputImage(cdst);
    }
    return 0;
}
