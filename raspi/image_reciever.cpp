#define VISION_HEADER __builtin_bswap32(0x6101FEED)
#define VISION_INFO __builtin_bswap32(0x6101DA7A)
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
//#define DEBUG

int serialOpen (const char *device, const int baud)
{
  struct termios options ;
  speed_t myBaud ;
  int     status, fd ;

  switch (baud)
  {
    case      50:	myBaud =      B50 ; break ;
    case      75:	myBaud =      B75 ; break ;
    case     110:	myBaud =     B110 ; break ;
    case     134:	myBaud =     B134 ; break ;
    case     150:	myBaud =     B150 ; break ;
    case     200:	myBaud =     B200 ; break ;
    case     300:	myBaud =     B300 ; break ;
    case     600:	myBaud =     B600 ; break ;
    case    1200:	myBaud =    B1200 ; break ;
    case    1800:	myBaud =    B1800 ; break ;
    case    2400:	myBaud =    B2400 ; break ;
    case    4800:	myBaud =    B4800 ; break ;
    case    9600:	myBaud =    B9600 ; break ;
    case   19200:	myBaud =   B19200 ; break ;
    case   38400:	myBaud =   B38400 ; break ;
    case   57600:	myBaud =   B57600 ; break ;
    case  115200:	myBaud =  B115200 ; break ;
    case  230400:	myBaud =  B230400 ; break ;
    case  460800:	myBaud =  B460800 ; break ;
    case  500000:	myBaud =  B500000 ; break ;
    case  576000:	myBaud =  B576000 ; break ;
    case  921600:	myBaud =  B921600 ; break ;
    case 1000000:	myBaud = B1000000 ; break ;
    case 1152000:	myBaud = B1152000 ; break ;
    case 1500000:	myBaud = B1500000 ; break ;
    case 2000000:	myBaud = B2000000 ; break ;
    case 2500000:	myBaud = B2500000 ; break ;
    case 3000000:	myBaud = B3000000 ; break ;
    case 3500000:	myBaud = B3500000 ; break ;
    case 4000000:	myBaud = B4000000 ; break ;

    default:
      return -2 ;
  }

  if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY)) == -1)
    return -1 ;

  fcntl (fd, F_SETFL, O_RDWR) ;

// Get and modify current options:

  tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);

  usleep (10000) ;	// 10mS

  return fd ;
}

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
    signal(SIGINT, atexit);
    signal(SIGKILL, atexit);
    signal(SIGTERM, atexit);
    signal(SIGHUP, atexit);
    tty_raw();
    fcntl (0, F_SETFL, O_NONBLOCK);
    int fd = serialOpen("/dev/ttyUSB0", 9600);
    if (fd < 3) {
        std::cerr << "Could not open serial port\n";
        atexit(SIGHUP);
    }
    std::cerr << "Press + or - to change threshold\n";
    while (true) {
        // Send threshold info if a key is pressed
        char c = 0;
        read(0, &c, 1);
        if (c == '=' || c == '+' || c == '-' || c == '_') write(fd, &c, 1);
        // Check if a frame is being sent and read it
        uint8_t bytes[16];
        uint8_t byte1 = 0, byte2 = 0;
        read(fd, &byte1, 1);
        if (byte1 != 0x61) {
            #ifdef DEBUG
            std::cout << "Bad byte " << (int)byte1 << "\n";
            #endif
            continue;
        }
        read(fd, &byte2, 1);
        if (byte2 != 0x01) {
            #ifdef DEBUG
            std::cout << "Bad byte 2 " << (int)byte2 << "\n";
            #endif
            continue;
        }
        bytes[0] = byte1;
        bytes[1] = byte2;
        for (int i = 2; i < 16; i++)
            read(fd, &bytes[i], 1);
        #ifdef DEBUG
        std::cout << "Recieved frame\n";
        #endif
        uint16_t * bytes16 = (uint16_t*)bytes;
        uint32_t * bytes32 = (uint32_t*)bytes;
        // Check the checksum
        #ifdef DEBUG
        for (int i = 0; i < 8; i++) 
            std::cout << std::hex << std::setw(4) << std::setfill('0') << __builtin_bswap16(bytes16[i]) << " ";
        std::cout << "\n";
        #endif
        uint32_t sum = bytes16[0] + bytes16[1] + bytes16[2] + bytes16[3] + 
                        bytes16[4] + bytes16[5] + bytes16[6] + bytes16[7];
        uint16_t csum = ~((uint16_t)(sum & 0xFFFF) + (uint16_t)(sum >> 16));
        if (csum != 0) {
            std::cerr << "Invalid checksum (got " << csum << "), discarding\n";
            continue;
        }
        if (bytes32[0] == VISION_INFO) {
            std::cout << "Threshold is now " << ((float*)bytes)[1] << "\n";
        } else if (bytes32[0] == VISION_HEADER) {
            float angle = ((float*)bytes32)[1];
            float cmDistance = ((float*)bytes32)[2];
            int16_t dist = bytes16[6];
            if (angle == 0 && cmDistance == 0 && dist == 0) continue;
            std::cout << "Angle = " << angle << "\u00B0, line distance = " << cmDistance << " cm, wall distance = " << dist << "\n";
        } else {
            std::cerr << "Invalid header, discarding\n";
        }
    }
}
