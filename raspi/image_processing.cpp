#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
//#include <png++/png.hpp>
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

int main() {
    // Set up capture
    VideoCapture capture(2);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    capture.set(CV_CAP_PROP_CONTRAST, 1.0);
    capture.set(CV_CAP_PROP_BRIGHTNESS, brightness);
    if(!capture.isOpened()){
        std::cerr << "Failed to connect to the camera.\n";
        return 1;
    }
    while (true) {
        Mat frame, edges(Size(640, 480), CV_8UC1), cdst(Size(640, 480*2), CV_8UC3);
        capture >> frame;
        if(frame.empty()){
            std::cerr << "Failed to capture an image.\n";
            return 2;
        }
        // Create PNG
        //png::image<png::rgb_pixel> img(640, 480*2);
        // Convert to B/W image
        std::vector<bool> lastRow(640, 0);
        for (int x = 0; x < frame.rows; x++) {
            bool lastValue = false;
            for (int y = 0; y < frame.cols; y++) {
                pixel_t pix = mpx(frame.at<Vec3b>(x, y).val);
                uchar avg = (pix.r + pix.g + pix.b) / 3;
                /*bool v = (avg >= pixel_threshold) != lastValue || 
                        (avg >= pixel_threshold) != lastRow[y] || 
                        (avg >= pixel_threshold && y == frame.cols - 1) ||
                        (avg >= pixel_threshold && x == frame.rows - 1);*/
                bool v = avg >= pixel_threshold;
                edges.at<uchar>(x, y) = v * 255;
                //img.set_pixel(y, x, png::rgb_pixel(v * 255, v * 255, v * 255));
                //img.set_pixel(y, x + 480, png::rgb_pixel(pix.r, pix.g, pix.b));
                cdst.at<Vec3b>(x + 480, y) = Vec3b(v * 255, v * 255, v * 255);
                cdst.at<Vec3b>(x, y) = Vec3b(pix.b, pix.g, pix.r);
                lastValue = avg >= pixel_threshold;
                lastRow[y] = lastValue;
            }
        }
        // Detect lines
        /*std::vector<Vec4i> lines;
        HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 50, 10);
        for (Vec4i line : lines) {
            point_t a, b, la = {line[0], line[1]}, lb = {line[2], line[3]};
            a.x = 0;
            a.y = sqrt((lb.x - la.x)^2 + (lb.y - la.y)^2);
            b.x = lb.x - la.x;
            b.y = lb.y - la.y;
            double angle = (atan2(a.y, a.x) - atan2(b.y, b.x)) * (180/M_PI);
            cv::line(cdst, Point(line[0], line[1] + 960), Point(line[2], line[3] + 960), Scalar(255, 255, 255), 3, 2);
            std::cout << "First point at (" << line[0] << ", " << line[1] << "), last point at (" << line[2] << ", " << line[3] << "), angle = " << angle << "\n";
        }*/
        // extract contours
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        if (contours.size() > 1) continue;
        for(int i=0; i<contours.size(); ++i)
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

            // read center of rotated rect
            cv::Point2f center = rotatedRect.center; // center

            // draw rotated rect
            for(unsigned int j=0; j<4; ++j)
                cv::line(cdst, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,255,0));

            // draw center and print text
            if (!isnan(angle - 90.0)) std::cout << angle - 90.0 << "\n";
            imshow("Rect", cdst);
            //std::stringstream ss;   ss << angle; // convert float to string
            //cv::circle(cdst, center, 5, cv::Scalar(0,255,0)); // draw center
            //cv::putText(cdst, ss.str(), center + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255)); // print angle
        }
    }
    //img.write("detected_image.png");
    //imwrite("lines.png", cdst);
    return 0;
}