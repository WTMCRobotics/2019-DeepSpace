// Vision reciever library
#define VISION_HEADER (0xEDFE0161)
#define VISION_INFO (0x7ADA0161)
#ifdef __GNUC__
#define DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif
#include <cstdint>
#include <string>
#include <frc/SerialPort.h>

enum vision_error {
    VISION_ERROR_SUCCESS = 0b00000000,
    VISION_ERROR_NO_DATA = 0b00000001,
    VISION_ERROR_BAD_RECT = 0b00000010,
    VISION_ERROR_BAD_DISTANCE = 0b00000100,
    VISION_ERROR_BAD_HEADER = 0b00001000,
    VISION_ERROR_BAD_CHECKSUM = 0b00010000,
    VISION_ERROR_BROKEN = 0b10000000
};

// Vision frame struct
typedef struct {
    uint32_t header;
    float angle;
    int16_t line_offset;
    int16_t line_offset_y;
    uint16_t wall_distance_left;
    uint16_t wall_distance_right;
    uint16_t checksum;
    uint8_t error;
} vision_frame_t;

// Vision info struct
typedef struct {
    uint32_t header;
    float threshold;
    char zero[8];
    uint16_t checksum;
    uint8_t error;
} vision_info_t;

// Returns a frame returned from the robot.
// DO NOT RELY ON THIS BEING A TRUE FRAME! Use frameIsInfo() to check.
extern vision_frame_t getFrame(frc::SerialPort &serial);

// Checks if the frame is an info frame.
#define frameIsInfo(frame) (frame.header == VISION_INFO)

// Converts a frame to an info frame struct.
// Redundant; vision_frame_t.angle is also the threshold.
extern vision_info_t getInfo(vision_frame_t frame);

// Increments the threshold and returns an info frame.
// If no vision system is active, this will hang.
extern vision_info_t incrementThreshold(frc::SerialPort &serial);

// Decrements the threshold and returns an info frame.
// If no vision system is active, this will hang.
extern vision_info_t decrementThreshold(frc::SerialPort &serial);

// Returns a string with a description of the error.
extern std::string getVisionError(vision_frame_t frame);

// Switches the camera output to the next camera.
#define nextCamera(serial) serial.Write("n", 1)

// Calibrates the vision threshold.
DEPRECATED(extern vision_info_t calibrateVision(frc::SerialPort &serial));