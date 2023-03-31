#include "SerialDriver.h"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <Logger.h>

bool setBaud(int fd, int baud) {
    int speed_arr[] = {B230400, B115200, B38400, B19200, B9600, B4800, B1200,
                       B300,    B38400,  B19200, B9600,  B4800, B1200, B300};
    int name_arr[] = {230400, 115200, 38400, 19200, 9600, 4800, 1200,
                      300,    38400,  19200, 9600,  4800, 1200, 300};

    struct termios Opt;
    tcgetattr(fd, &Opt);
    for (int i = 0; i < (sizeof(speed_arr) / sizeof(int)); i++) {
        if (baud == name_arr[i]) {
            tcflush(fd, TCIOFLUSH);
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            int status = tcsetattr(fd, TCSANOW, &Opt);
            if (status != 0) {
                return false;
            }
            tcflush(fd, TCIOFLUSH);
            break;
        }
    }

    return true;
}

bool setMode(int fd, int databits, int stopbits, char parity) {
    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        return false;
    }
    options.c_cflag &= ~CSIZE;
    switch (databits) {
        case 7:
        options.c_cflag |= CS7;
        break;
        case 8:
        options.c_cflag |= CS8;
        break;
        default:
        LOG_ERROR("Unsupported data size: {}", databits);
        return false;
    }

    switch (parity) {
        case 'n':
        case 'N':
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
        case 'o':
        case 'O':
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
        case 'e':
        case 'E':
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
        case 'S':
        case 's':
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
        default:
        LOG_ERROR("Unsupported parity: {}", parity);
        return false;
    }

    switch (stopbits) {
        case 1:
        options.c_cflag &= ~CSTOPB;
        break;
        case 2:
        options.c_cflag |= CSTOPB;
        break;
        default:
        LOG_ERROR("Unsupported stop bits: {}", stopbits);
        return false;
    }

    if (parity != 'n') options.c_iflag |= INPCK;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // 141111: Binary mode to disable auto adding '0d' before '0a'
    options.c_oflag &= ~(INLCR | IGNCR | ICRNL);
    options.c_oflag &= ~(ONLCR | OCRNL);

    tcflush(fd, TCIFLUSH);
    options.c_cc[VTIME] = 150;
    options.c_cc[VMIN] = 0;
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        return false;
    }

    return true;
}

SerialDriver::SerialDriver(std::string device, int baud) {
    mDevice = device;
    mBaud = baud;
    mFd = -1;
}

SerialDriver::~SerialDriver() {
    closeSerial();
}

bool SerialDriver::openSerial() {
    mFd = open(mDevice.c_str(), O_RDWR);
    if (mFd == -1) {
        LOG_DEBUG("can not open device: {}", mDevice);
        return false;
    }
    LOG_INFO("open device success: {}", mDevice);

    bool success = setBaud(mFd, mBaud);
    if (!success) {
        LOG_ERROR("{} : can not set baud {}", mDevice, mBaud);
        mFd = -1;
        return false;
    }
    LOG_INFO("{} : set baud {}, success", mDevice, mBaud);

    int databits = 8;
    int stopbits = 1;
    char parity = 'N';
    success = setMode(mFd, databits, stopbits, parity);
    if (!success) {
        LOG_ERROR("{} : can not set mode {}, {}, {}", mDevice, databits, stopbits, parity);
        mFd = -1;
        return false;
    }
    tcflush(mFd, TCIOFLUSH);

    LOG_INFO("The {} is working at: {}, {}, {}", mDevice, databits, stopbits, parity);
    return true;
}

void SerialDriver::closeSerial() {
    if (mFd != -1) {
        close(mFd);
        mFd = -1;
        LOG_INFO("close device success: {}", mDevice);
    }
}

int SerialDriver::readBuf(char buf[], int length, int timeout) {
    if (mFd == -1) {
        bool success = openSerial();
        if (!success) {
            usleep(timeout);
            return 0;
        }
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(mFd, &fds);
    struct timeval tv;
    tv.tv_sec = timeout / 1000000;
    tv.tv_usec = timeout % 1000000;
    int ret = select(mFd + 1, &fds, NULL, NULL, &tv);

    if(!FD_ISSET(mFd, &fds)) {
        LOG_WARN("device: {}, select timeout {} ms", mDevice, timeout / 1000.0);
        return 0;
    }

    int size = read(mFd, buf, length);
    if (size == 0) {
        LOG_WARN("device: {}, read no data", mDevice);
        closeSerial();
        return 0;
    }

    return size;
}