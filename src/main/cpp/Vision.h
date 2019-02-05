// Vision reciever library
#define VISION_HEADER __builtin_bswap32(0x6101FEED)
#define VISION_INFO __builtin_bswap32(0x6101DA7A)
#include <cstdint>
#include <frc/SerialPort.h>

enum vision_error {
    VISION_ERROR_SUCCESS = 0,
    VISION_ERROR_NO_DATA = 1,
    VISION_ERROR_BAD_RECT = 2,
    VISION_ERROR_BAD_DISTANCE = 4
};

// Vision frame struct
typedef struct {
    uint32_t header;
    float angle;
    float line_offset;
    int16_t wall_distance;
    uint16_t checksum;
    uint8_t error;
} vision_frame_t;

// Vision info struct
typedef struct {
    uint32_t header;
    float threshold;
    char zero[6];
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