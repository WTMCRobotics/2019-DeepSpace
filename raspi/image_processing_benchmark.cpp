#define VISION_HEADER __builtin_bswap32(0x6101FEED)
#define VISION_INFO __builtin_bswap32(0x6101DA7A)
#include <iostream>
#include <vector>
#include <cstdint>
#include <csignal>
#include <chrono>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std::chrono;

//#define CUDA // define to use GPU processing

#ifdef CUDA
#include <opencv2/gpu/gpu.hpp>
#endif

uchar pixel_threshold = 160;
double brightness = 0.25;
steady_clock::time_point t;
std::vector<int> times;

void outputImage(bool success) {
    if (success) {
        std::cout << "Got frame in " << (duration_cast<milliseconds>(steady_clock::now() - t)).count() << "ms\n";
        times.push_back((duration_cast<milliseconds>(steady_clock::now() - t)).count());
    } else std::cout << "Failed to get frame\n";
}

int main() {
    // Set up capture
    VideoCapture capture("benchmark.avi");
    if(!capture.isOpened()) {
        std::cerr << "Failed to connect to the camera.\n";
        return 1;
    }
    #ifdef CUDA
    std::cout << "CUDA is enabled. ";
    #endif
    std::cout << "Starting benchmark.\n";
    while (true) {
        t = steady_clock::now();
        uint32_t frames[4];
        frames[0] = VISION_HEADER;
        Mat frame, edges;
        #ifdef CUDA
        cv::gpu::GpuMat gpuframe, bw, gpuedges;
        #else
        Mat bw;
        #endif
        capture >> frame;
        if(frame.empty()) break;
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
        uchar thresh = ((max - min) * (pixel_threshold / 256.0)) + min;
        // Convert to B/W image
        #ifdef CUDA
        cv::gpu::threshold(bw, gpuedges, (double)thresh, 255.0, THRESH_BINARY);
        gpuedges.download(edges);
        #else
        threshold(bw, edges, (double)thresh, 255.0, THRESH_BINARY);
        #endif
        // extract contours
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        if (contours.size() < 1) {outputImage(false); continue;}
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
        if (angle == 90.0 || angle == 0.0-90.0 || angle == 0.0 || angle == 45.0 || angle == 0.0-45.0) {outputImage(false); continue;}
        // read center of rotated rect
        cv::Point2f center = rotatedRect.center; // center

        // draw rotated rect
        if (isnan(angle)) {outputImage(false); continue;}
        frames[1] = *(uint32_t*)(&angle);
        
        // Calculate offset
        float multiplier = (center.x - 320.0) / sqrt(pow(usedEdge.x, 2) + pow(usedEdge.y, 2));
        // Game manual says the length of the tape is 2"/5.08 cm,
        // we can find the distance from the center using the multiplier
        float cmDistance = multiplier * 5.08;
        float inchDistance = multiplier * 2.0;
        frames[2] = *(uint32_t*)(&cmDistance);
        
        // TODO: get distance data
        uint16_t * frames16 = (uint16_t*)frames;
        frames16[6] = 0;

        // Calculate checksum
        uint32_t sum = frames16[0] + frames16[1] + frames16[2] + 
        frames16[3] + frames16[4] + frames16[5] + frames16[6];
        frames16[7] = ~((uint16_t)(sum & 0xFFFF) + (uint16_t)(sum >> 16));

        // send data out
        std::cout << std::hex;
        for (int i = 0; i < 16; ++i)
            std::cout << std::setfill('0') << std::setw(2) << (int)((char*)frames)[i] << " ";
        std::cout << std::dec << std::endl;

        // output final image
        outputImage(true);
    }
    int avg = 0;
    for (int i : times) avg += i;
    avg /= times.size();
    std::cout << "Total frames = " << times.size() << ", average time = " << avg << " ms\n";
    return 0;
}
