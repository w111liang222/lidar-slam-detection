#ifndef __SERIAL_DRIVER_H
#define __SERIAL_DRIVER_H

#include <string>

class SerialDriver {
 public:
  SerialDriver(std::string device, int baud);
  virtual ~SerialDriver();
  int readBuf(char buf[], int length, int timeout);

 protected:
  bool openSerial();
  void closeSerial();

 private:
  std::string mDevice;
  int mBaud;
  int mFd;
};

#endif // __SERIAL_DRIVER_H