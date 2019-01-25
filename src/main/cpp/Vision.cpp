#include "Vision.h"

vision_frame_t getFrame(frc::SerialPort * serial) {
    vision_frame_t frame;
    while (true) {
        char bytes[16];
        char byte1 = 0, byte2 = 0;
        // Scan for header, don't read until it aligns
        serial->Read(&byte1, 1);
        if (byte1 != 0x61) {
            #ifdef DEBUG
            std::cout << "Bad byte " << (int)byte1 << "\n";
            #endif
            continue;
        }
        serial->Read(&byte2, 1);
        if (byte2 != 0x01) {
            #ifdef DEBUG
            std::cout << "Bad byte 2 " << (int)byte2 << "\n";
            #endif
            continue;
        }
        // It aligns, read the rest
        ((char*)&frame)[0] = byte1;
        ((char*)&frame)[1] = byte2;
        for (int i = 2; i < 16; i++)
            serial->Read(&((char*)&frame)[i], 1);    
        return frame;
    }
}

vision_info_t getInfo(vision_frame_t frame) {
    return *((vision_info_t*)&frame);
}

vision_info_t incrementThreshold(frc::SerialPort * serial) {
    serial->Write("+", 1);
    while (true) {
        vision_frame_t frame = getFrame(serial);
        if (frameIsInfo(frame)) return getInfo(frame);
    }
}

vision_info_t decrementThreshold(frc::SerialPort * serial) {
    serial->Write("-", 1);
    while (true) {
        vision_frame_t frame = getFrame(serial);
        if (frameIsInfo(frame)) return getInfo(frame);
    }
}
