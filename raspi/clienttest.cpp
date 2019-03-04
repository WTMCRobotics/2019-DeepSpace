#include <iostream>
#include <thread>
#include <mutex>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

extern int clientMain(const char * hostname, int portno, char * buffer, bool * ready, std::mutex * mtx);

int main(int argc, const char *argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <hostname> <port>\n";
        return 1;
    }
    char buffer[256];
    bool ready = false;
    std::mutex mtx;
    std::thread clientThread(clientMain, argv[1], atoi(argv[2]), buffer, &ready, &mtx);
    clientThread.detach();
    usleep(1000);
    while (true) {
        mtx.lock();
        std::cout << "Locked main thread\n";
        std::cin.read(buffer, 256);
        ready = true;
        std::cout << "Unlocking main thread\n";
        mtx.unlock();
        while (ready);
        //usleep(1000);
    }
    return 0;
}
