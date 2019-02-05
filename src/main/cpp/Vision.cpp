#include "Vision.h"
#include <iostream>
//#define VISION_DEBUG

vision_frame_t getFrame(frc::SerialPort &serial) {
    vision_frame_t frame;
    if (serial.StatusIsFatal()) {
        std::cout << "Serial is not available\n";
        return frame;
    }
    while (true) {
        char bytes[16];
        char byte1 = 0, byte2 = 0;
        // Scan for header, don't read until it aligns
        serial.Read(&byte1, 1);
        if (byte1 != 0x61) {
            #ifdef VISION_DEBUG
            std::cout << "Bad byte " << (int)byte1 << "\n";
            #endif
            if (byte1 == 0) {
                frame.error = VISION_ERROR_NO_DATA;
                return frame;
            }
            continue;
        }
        serial.Read(&byte2, 1);
        if (byte2 != 0x01) {
            #ifdef VISION_DEBUG
            std::cout << "Bad byte 2 " << (int)byte2 << "\n";
            #endif
            if (byte2 == 0) {
                frame.error = VISION_ERROR_NO_DATA;
                return frame;
            }
            continue;
        }
        // It aligns, read the rest
        ((char*)&frame)[0] = byte1;
        ((char*)&frame)[1] = byte2;
        for (int i = 2; i < 16; i++)
            serial.Read(&((char*)&frame)[i], 1);    
        if (frame.angle == 0 || frame.line_offset == 0) frame.error |= VISION_ERROR_BAD_RECT;
        if (frame.wall_distance == -1 || frame.wall_distance == 0) frame.error |= VISION_ERROR_BAD_DISTANCE;
        return frame;
    }
}

vision_info_t getInfo(vision_frame_t frame) {
    return *((vision_info_t*)&frame);
}

vision_info_t incrementThreshold(frc::SerialPort &serial) {
    serial.Write("+", 1);
    while (true) {
        vision_frame_t frame = getFrame(serial);
        if (frameIsInfo(frame)) return getInfo(frame);
    }
}

vision_info_t decrementThreshold(frc::SerialPort &serial) {
    serial.Write("-", 1);
    while (true) {
        vision_frame_t frame = getFrame(serial);
        if (frameIsInfo(frame)) return getInfo(frame);
    }
}
