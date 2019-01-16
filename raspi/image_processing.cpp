#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <csignal>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
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

uchar pixel_threshold = 192-32;
double brightness = 0.25;

static struct termios orig_termios;  /* TERMinal I/O Structure */
static int ttyfd = STDIN_FILENO;     /* STDIN_FILENO is 0 by default */
/* put terminal in raw mode - see termio(7I) for modes */
void tty_raw(void)
   {
    struct termios new_tio;
	unsigned char c;

	/* get the terminal settings for stdin */
	tcgetattr(STDIN_FILENO,&orig_termios);

	/* we want to keep the old setting to restore them a the end */
	new_tio=orig_termios;

	/* disable canonical mode (buffered i/o) and local echo */
	new_tio.c_lflag &=(~ICANON & ~ECHO);

	/* set the new settings immediately */
	tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);
   }

void atexit(int signal) {
    tcsetattr(ttyfd,TCSAFLUSH,&orig_termios);
    exit(10);
}

int main() {
    // Set up capture
    VideoCapture capture(0);
    VideoWriter output("output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 15, Size(640, 480*2));
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    capture.set(CV_CAP_PROP_CONTRAST, 1.0);
    capture.set(CV_CAP_PROP_BRIGHTNESS, brightness);
    if(!capture.isOpened()){
        std::cerr << "Failed to connect to the camera.\n";
        return 1;
    }
    signal(SIGINT, atexit);
    signal(SIGKILL, atexit);
    signal(SIGTERM, atexit);
    signal(SIGHUP, atexit);
    tty_raw();
    fcntl (0, F_SETFL, O_NONBLOCK);
    std::cerr << "Press + or - to change threshold\n";
    while (true) {
        Mat frame, edges(Size(640, 480), CV_8UC1), denoised, cdst(Size(640, 480*2), CV_8UC3);
        capture >> frame;
        if(frame.empty()){
            std::cerr << "Failed to capture an image.\n";
            return 2;
        }
        char c = 0;
        ::read(0, &c, 1);
        if (c == '=' || c == '+') std::cerr << "Threshold now " << (int)++pixel_threshold << "\n";
        else if (c == '-' || c == '_') std::cerr << "Threshold now " << (int)--pixel_threshold << "\n";
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
        if (contours.size() < 1) continue;
        //for(int i=0; i<1; ++i)
        int i = 0;
        {
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
            if (angle == 90.0 || angle == -90.0 || angle == 0.0) continue;
            // read center of rotated rect
            cv::Point2f center = rotatedRect.center; // center

            // draw rotated rect
            for(unsigned int j=0; j<4; ++j)
                cv::line(cdst, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,255,0));

            // draw center and print text
            if (!isnan(angle - 90.0)) std::cerr << angle - 90.0 << "\n";
            std::vector<uchar> buf;
            imencode(".jpg", cdst, buf);
            std::cout << std::string((char*)&buf[0], buf.size());
        }
    }
    return 0;
}