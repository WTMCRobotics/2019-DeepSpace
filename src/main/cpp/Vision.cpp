#include "Vision.h"
#include <iostream>
#include <iomanip>
#include <bitset>
#define VISION_DEBUG

vision_frame_t getFrame(frc::SerialPort &serial) {
    vision_frame_t frame;
    frame.error = 0;
    if (serial.StatusIsFatal()) {
        //std::cout << "Serial is not available\n";
        return frame;
    }
    while (true) {
        char bytes[16];
        char byte1 = 0, byte2 = 0;
        // Scan for header, don't read until it aligns
        serial.Read(&byte1, 1);
        //std::cout << std::hex << (int)byte1 << " ";
        if (byte1 != 0x61) {
            #ifdef VISION_DEBUG
            //std::cout << "Bad byte " << (int)byte1 << "\n";
            #endif
            if (byte1 == 0) {
                frame.error = VISION_ERROR_NO_DATA;
                //std::cout << std::dec;
                return frame;
            }
            continue;
        }
        serial.Read(&byte2, 1);
        //std::cout << (int)byte2 << " ";
        if (byte2 != 0x01) {
            #ifdef VISION_DEBUG
            std::cout << "Bad byte 2 " << (int)byte2 << "\n";
            #endif
            if (byte2 == 0) {
                frame.error = VISION_ERROR_NO_DATA;
                //std::cout << std::dec;
                return frame;
            }
            continue;
        }
        // It aligns, read the rest
        ((char*)&frame)[0] = byte1;
        ((char*)&frame)[1] = byte2;
        for (int i = 2; i < 18; i++) {
            serial.Read(&((char*)&frame)[i], 1);
            //std::cout << (int)((char*)&frame)[i] << " ";
        }
        if (frame.header != VISION_HEADER && frame.header != VISION_INFO) frame.error |= VISION_ERROR_BAD_HEADER;  
        if (frame.angle == 0 || frame.line_offset == 0 || frame.line_offset_y == 0) frame.error |= VISION_ERROR_BAD_RECT;
        if (frame.wall_distance_left > 325 || frame.wall_distance_left == 0) frame.error |= VISION_ERROR_BAD_DISTANCE;
        uint16_t * frames16 = (uint16_t*)&frame;
        //std::cout << "\n";
        //for (int i = 0; i < 8; i++) std::cout << frames16[i] << " ";
        if (frames16[0] != 0x6101 && frames16[0] != 0x0161) frame.error |= VISION_ERROR_BROKEN;
        uint32_t subsum = frames16[0] + frames16[1] + frames16[2] + frames16[3] +
            frames16[4] + frames16[5] + frames16[6] + frames16[7] + frames16[8];
        uint16_t sum = ~((uint16_t)(subsum & 0xFFFF) + (uint16_t)(subsum >> 16));
        //std::cout << "\n" << subsum << " " << sum;
        if (sum != 0) frame.error |= VISION_ERROR_BAD_CHECKSUM;
        //std::cout << std::dec << " " << std::bitset<8>(frame.error) << "\n";
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

std::string getVisionError(vision_frame_t frame) {
    if (frame.error == 0) return "Success";
    std::string retval = "";
    if (frame.error & VISION_ERROR_NO_DATA) retval += ", No data recieved";
    if (frame.error & VISION_ERROR_BAD_RECT) retval += ", Bad rectangle";
    if (frame.error & VISION_ERROR_BAD_HEADER) retval += ", Bad header";
    if (frame.error & VISION_ERROR_BAD_DISTANCE) retval += ", Bad ultrasonic distance";
    if (frame.error & VISION_ERROR_BAD_CHECKSUM) retval += ", Bad checksum";
    if (frame.error & VISION_ERROR_BROKEN) retval += ", Invalid error (this should never occur)";
    return retval.substr(2);
}

vision_info_t calibrateVision(frc::SerialPort &serial) {
    serial.Write("c", 1);
    while (true) {
        vision_frame_t frame = getFrame(serial);
        if (frameIsInfo(frame)) return getInfo(frame);
    }
}
