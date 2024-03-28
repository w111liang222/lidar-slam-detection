#include <algorithm>
# include <alloca.h>

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/signal.h>
#include <errno.h>
#include <paths.h>
#include <sysexits.h>
#include <termios.h>
#include <sys/param.h>
# include <linux/serial.h>

#include <sys/select.h>
#include <sys/time.h>
#include <time.h>
#include <Logger.h>

#ifndef TIOCINQ
#ifdef FIONREAD
#define TIOCINQ FIONREAD
#else
#define TIOCINQ 0x541B
#endif
#endif

#include "SerialDriver.h"

using std::invalid_argument;
using std::min;
using std::numeric_limits;
using std::vector;
using std::size_t;
using std::string;
using std::stringstream;

using serial::Serial;
using serial::MillisecondTimer;
using serial::bytesize_t;
using serial::parity_t;
using serial::stopbits_t;
using serial::flowcontrol_t;

class Serial::ScopedReadLock {
public:
  ScopedReadLock(SerialImpl *pimpl) : pimpl_(pimpl) {
    this->pimpl_->readLock();
  }
  ~ScopedReadLock() {
    this->pimpl_->readUnlock();
  }
private:
  // Disable copy constructors
  ScopedReadLock(const ScopedReadLock&);
  const ScopedReadLock& operator=(ScopedReadLock);

  SerialImpl *pimpl_;
};

class Serial::ScopedWriteLock {
public:
  ScopedWriteLock(SerialImpl *pimpl) : pimpl_(pimpl) {
    this->pimpl_->writeLock();
  }
  ~ScopedWriteLock() {
    this->pimpl_->writeUnlock();
  }
private:
  // Disable copy constructors
  ScopedWriteLock(const ScopedWriteLock&);
  const ScopedWriteLock& operator=(ScopedWriteLock);
  SerialImpl *pimpl_;
};

Serial::Serial (const string &port, uint32_t baudrate, serial::Timeout timeout,
                bytesize_t bytesize, parity_t parity, stopbits_t stopbits,
                flowcontrol_t flowcontrol)
 : pimpl_(new SerialImpl (port, baudrate, bytesize, parity,
                                           stopbits, flowcontrol))
{
  pimpl_->setTimeout(timeout);
}

Serial::~Serial ()
{
  delete pimpl_;
}

void
Serial::open ()
{
  pimpl_->open ();
}

void
Serial::close ()
{
  pimpl_->close ();
}

bool
Serial::isOpen () const
{
  return pimpl_->isOpen ();
}

size_t
Serial::available ()
{
  return pimpl_->available ();
}

bool
Serial::waitReadable ()
{
  serial::Timeout timeout(pimpl_->getTimeout ());
  return pimpl_->waitReadable(timeout.read_timeout_constant);
}

void
Serial::waitByteTimes (size_t count)
{
  pimpl_->waitByteTimes(count);
}

size_t
Serial::read_ (uint8_t *buffer, size_t size)
{
  return this->pimpl_->read (buffer, size);
}

size_t
Serial::read (uint8_t *buffer, size_t size)
{
  ScopedReadLock lock(this->pimpl_);
  return this->pimpl_->read (buffer, size);
}

size_t
Serial::read (std::vector<uint8_t> &buffer, size_t size)
{
  ScopedReadLock lock(this->pimpl_);
  uint8_t *buffer_ = new uint8_t[size];
  size_t bytes_read = 0;

  try {
    bytes_read = this->pimpl_->read (buffer_, size);
  }
  catch (const std::exception &e) {
    delete[] buffer_;
    throw;
  }

  buffer.insert (buffer.end (), buffer_, buffer_+bytes_read);
  delete[] buffer_;
  return bytes_read;
}

size_t
Serial::read (std::string &buffer, size_t size)
{
  ScopedReadLock lock(this->pimpl_);
  uint8_t *buffer_ = new uint8_t[size];
  size_t bytes_read = 0;
  try {
    bytes_read = this->pimpl_->read (buffer_, size);
  }
  catch (const std::exception &e) {
    delete[] buffer_;
    throw;
  }
  buffer.append (reinterpret_cast<const char*>(buffer_), bytes_read);
  delete[] buffer_;
  return bytes_read;
}

string
Serial::read (size_t size)
{
  std::string buffer;
  this->read (buffer, size);
  return buffer;
}

size_t
Serial::readline (string &buffer, size_t size, string eol)
{
  ScopedReadLock lock(this->pimpl_);
  size_t eol_len = eol.length ();
  uint8_t *buffer_ = static_cast<uint8_t*>
                              (alloca (size * sizeof (uint8_t)));
  size_t read_so_far = 0;
  while (true)
  {
    size_t bytes_read = this->read_ (buffer_ + read_so_far, 1);
    read_so_far += bytes_read;
    if (bytes_read == 0) {
      break; // Timeout occured on reading 1 byte
    }
    if(read_so_far < eol_len) continue;
    if (string (reinterpret_cast<const char*>
         (buffer_ + read_so_far - eol_len), eol_len) == eol) {
      break; // EOL found
    }
    if (read_so_far == size) {
      break; // Reached the maximum read length
    }
  }
  buffer.append(reinterpret_cast<const char*> (buffer_), read_so_far);
  return read_so_far;
}

string
Serial::readline (size_t size, string eol)
{
  std::string buffer;
  this->readline (buffer, size, eol);
  return buffer;
}

vector<string>
Serial::readlines (size_t size, string eol)
{
  ScopedReadLock lock(this->pimpl_);
  std::vector<std::string> lines;
  size_t eol_len = eol.length ();
  uint8_t *buffer_ = static_cast<uint8_t*>
    (alloca (size * sizeof (uint8_t)));
  size_t read_so_far = 0;
  size_t start_of_line = 0;
  while (read_so_far < size) {
    size_t bytes_read = this->read_ (buffer_+read_so_far, 1);
    read_so_far += bytes_read;
    if (bytes_read == 0) {
      if (start_of_line != read_so_far) {
        lines.push_back (
          string (reinterpret_cast<const char*> (buffer_ + start_of_line),
            read_so_far - start_of_line));
      }
      break; // Timeout occured on reading 1 byte
    }
    if(read_so_far < eol_len) continue;
    if (string (reinterpret_cast<const char*>
         (buffer_ + read_so_far - eol_len), eol_len) == eol) {
      // EOL found
      lines.push_back(
        string(reinterpret_cast<const char*> (buffer_ + start_of_line),
          read_so_far - start_of_line));
      start_of_line = read_so_far;
    }
    if (read_so_far == size) {
      if (start_of_line != read_so_far) {
        lines.push_back(
          string(reinterpret_cast<const char*> (buffer_ + start_of_line),
            read_so_far - start_of_line));
      }
      break; // Reached the maximum read length
    }
  }
  return lines;
}

size_t
Serial::write (const string &data)
{
  ScopedWriteLock lock(this->pimpl_);
  return this->write_ (reinterpret_cast<const uint8_t*>(data.c_str()),
                       data.length());
}

size_t
Serial::write (const std::vector<uint8_t> &data)
{
  ScopedWriteLock lock(this->pimpl_);
  return this->write_ (&data[0], data.size());
}

size_t
Serial::write (const uint8_t *data, size_t size)
{
  ScopedWriteLock lock(this->pimpl_);
  return this->write_(data, size);
}

size_t
Serial::write_ (const uint8_t *data, size_t length)
{
  return pimpl_->write (data, length);
}

void
Serial::setPort (const string &port)
{
  ScopedReadLock rlock(this->pimpl_);
  ScopedWriteLock wlock(this->pimpl_);
  bool was_open = pimpl_->isOpen ();
  if (was_open) close();
  pimpl_->setPort (port);
  if (was_open) open ();
}

string
Serial::getPort () const
{
  return pimpl_->getPort ();
}

void
Serial::setTimeout (serial::Timeout &timeout)
{
  pimpl_->setTimeout (timeout);
}

serial::Timeout
Serial::getTimeout () const {
  return pimpl_->getTimeout ();
}

void
Serial::setBaudrate (uint32_t baudrate)
{
  pimpl_->setBaudrate (baudrate);
}

uint32_t
Serial::getBaudrate () const
{
  return uint32_t(pimpl_->getBaudrate ());
}

void
Serial::setBytesize (bytesize_t bytesize)
{
  pimpl_->setBytesize (bytesize);
}

bytesize_t
Serial::getBytesize () const
{
  return pimpl_->getBytesize ();
}

void
Serial::setParity (parity_t parity)
{
  pimpl_->setParity (parity);
}

parity_t
Serial::getParity () const
{
  return pimpl_->getParity ();
}

void
Serial::setStopbits (stopbits_t stopbits)
{
  pimpl_->setStopbits (stopbits);
}

stopbits_t
Serial::getStopbits () const
{
  return pimpl_->getStopbits ();
}

void
Serial::setFlowcontrol (flowcontrol_t flowcontrol)
{
  pimpl_->setFlowcontrol (flowcontrol);
}

flowcontrol_t
Serial::getFlowcontrol () const
{
  return pimpl_->getFlowcontrol ();
}

void Serial::flush ()
{
  ScopedReadLock rlock(this->pimpl_);
  ScopedWriteLock wlock(this->pimpl_);
  pimpl_->flush ();
}

void Serial::flushInput ()
{
  ScopedReadLock lock(this->pimpl_);
  pimpl_->flushInput ();
}

void Serial::flushOutput ()
{
  ScopedWriteLock lock(this->pimpl_);
  pimpl_->flushOutput ();
}

MillisecondTimer::MillisecondTimer (const uint32_t millis)
  : expiry(timespec_now())
{
  int64_t tv_nsec = expiry.tv_nsec + (millis * 1e6);
  if (tv_nsec >= 1e9) {
    int64_t sec_diff = tv_nsec / static_cast<int> (1e9);
    expiry.tv_nsec = tv_nsec % static_cast<int>(1e9);
    expiry.tv_sec += sec_diff;
  } else {
    expiry.tv_nsec = tv_nsec;
  }
}

int64_t
MillisecondTimer::remaining ()
{
  timespec now(timespec_now());
  int64_t millis = (expiry.tv_sec - now.tv_sec) * 1e3;
  millis += (expiry.tv_nsec - now.tv_nsec) / 1e6;
  return millis;
}

timespec
MillisecondTimer::timespec_now ()
{
  timespec time;
# ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  time.tv_sec = mts.tv_sec;
  time.tv_nsec = mts.tv_nsec;
# else
  clock_gettime(CLOCK_MONOTONIC, &time);
# endif
  return time;
}

timespec
timespec_from_ms (const uint32_t millis)
{
  timespec time;
  time.tv_sec = millis / 1e3;
  time.tv_nsec = (millis - (time.tv_sec * 1e3)) * 1e6;
  return time;
}

Serial::SerialImpl::SerialImpl (const string &port, unsigned long baudrate,
                                bytesize_t bytesize,
                                parity_t parity, stopbits_t stopbits,
                                flowcontrol_t flowcontrol)
  : port_ (port), fd_ (-1), is_open_ (false), xonxoff_ (false), rtscts_ (false),
    baudrate_ (baudrate), parity_ (parity),
    bytesize_ (bytesize), stopbits_ (stopbits), flowcontrol_ (flowcontrol)
{
  pthread_mutex_init(&this->read_mutex, NULL);
  pthread_mutex_init(&this->write_mutex, NULL);
  if (port_.empty () == false)
    open ();
}

Serial::SerialImpl::~SerialImpl ()
{
  close();
  pthread_mutex_destroy(&this->read_mutex);
  pthread_mutex_destroy(&this->write_mutex);
}

void
Serial::SerialImpl::open ()
{
  if (port_.empty ()) {
    LOG_WARN("Empty serial port is invalid.");
    is_open_ = false;
    return;
  }
  if (is_open_ == true) {
    LOG_WARN("Serial port already open.");
    return;
  }

  fd_ = ::open (port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_ == -1) {
    LOG_DEBUG("can not open device: {}", port_);
    is_open_ = false;
    return;
  }

  is_open_ = reconfigurePort();
}

bool
Serial::SerialImpl::reconfigurePort ()
{
  struct termios options; // The options for the file descriptor

  if (tcgetattr(fd_, &options) == -1) {
    LOG_DEBUG("reconfigure serial port ::tcgetattr failed");
    return false;
  }

  // set up raw mode / no echo / binary
  options.c_cflag |= (tcflag_t)  (CLOCAL | CREAD);
  options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                       ISIG | IEXTEN); //|ECHOPRT

  options.c_oflag &= (tcflag_t) ~(OPOST);
  options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);
#ifdef IUCLC
  options.c_iflag &= (tcflag_t) ~IUCLC;
#endif
#ifdef PARMRK
  options.c_iflag &= (tcflag_t) ~PARMRK;
#endif

  // setup baud rate
  bool custom_baud = false;
  speed_t baud;
  switch (baudrate_) {
#ifdef B0
  case 0: baud = B0; break;
#endif
#ifdef B50
  case 50: baud = B50; break;
#endif
#ifdef B75
  case 75: baud = B75; break;
#endif
#ifdef B110
  case 110: baud = B110; break;
#endif
#ifdef B134
  case 134: baud = B134; break;
#endif
#ifdef B150
  case 150: baud = B150; break;
#endif
#ifdef B200
  case 200: baud = B200; break;
#endif
#ifdef B300
  case 300: baud = B300; break;
#endif
#ifdef B600
  case 600: baud = B600; break;
#endif
#ifdef B1200
  case 1200: baud = B1200; break;
#endif
#ifdef B1800
  case 1800: baud = B1800; break;
#endif
#ifdef B2400
  case 2400: baud = B2400; break;
#endif
#ifdef B4800
  case 4800: baud = B4800; break;
#endif
#ifdef B7200
  case 7200: baud = B7200; break;
#endif
#ifdef B9600
  case 9600: baud = B9600; break;
#endif
#ifdef B14400
  case 14400: baud = B14400; break;
#endif
#ifdef B19200
  case 19200: baud = B19200; break;
#endif
#ifdef B28800
  case 28800: baud = B28800; break;
#endif
#ifdef B57600
  case 57600: baud = B57600; break;
#endif
#ifdef B76800
  case 76800: baud = B76800; break;
#endif
#ifdef B38400
  case 38400: baud = B38400; break;
#endif
#ifdef B115200
  case 115200: baud = B115200; break;
#endif
#ifdef B128000
  case 128000: baud = B128000; break;
#endif
#ifdef B153600
  case 153600: baud = B153600; break;
#endif
#ifdef B230400
  case 230400: baud = B230400; break;
#endif
#ifdef B256000
  case 256000: baud = B256000; break;
#endif
#ifdef B460800
  case 460800: baud = B460800; break;
#endif
#ifdef B500000
  case 500000: baud = B500000; break;
#endif
#ifdef B576000
  case 576000: baud = B576000; break;
#endif
#ifdef B921600
  case 921600: baud = B921600; break;
#endif
#ifdef B1000000
  case 1000000: baud = B1000000; break;
#endif
#ifdef B1152000
  case 1152000: baud = B1152000; break;
#endif
#ifdef B1500000
  case 1500000: baud = B1500000; break;
#endif
#ifdef B2000000
  case 2000000: baud = B2000000; break;
#endif
#ifdef B2500000
  case 2500000: baud = B2500000; break;
#endif
#ifdef B3000000
  case 3000000: baud = B3000000; break;
#endif
#ifdef B3500000
  case 3500000: baud = B3500000; break;
#endif
#ifdef B4000000
  case 4000000: baud = B4000000; break;
#endif
  default:
    custom_baud = true;
  }
  if (custom_baud == false) {
#ifdef _BSD_SOURCE
    ::cfsetspeed(&options, baud);
#else
    ::cfsetispeed(&options, baud);
    ::cfsetospeed(&options, baud);
#endif
  }

  // setup char len
  options.c_cflag &= (tcflag_t) ~CSIZE;
  if (bytesize_ == eightbits)
    options.c_cflag |= CS8;
  else if (bytesize_ == sevenbits)
    options.c_cflag |= CS7;
  else if (bytesize_ == sixbits)
    options.c_cflag |= CS6;
  else if (bytesize_ == fivebits)
    options.c_cflag |= CS5;
  else
    LOG_DEBUG("reconfigure serial port invalid char len");
  // setup stopbits
  if (stopbits_ == stopbits_one)
    options.c_cflag &= (tcflag_t) ~(CSTOPB);
  else if (stopbits_ == stopbits_one_point_five)
    // ONE POINT FIVE same as TWO.. there is no POSIX support for 1.5
    options.c_cflag |=  (CSTOPB);
  else if (stopbits_ == stopbits_two)
    options.c_cflag |=  (CSTOPB);
  else
    LOG_DEBUG("reconfigure serial port invalid stop bit");
  // setup parity
  options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
  if (parity_ == parity_none) {
    options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);
  } else if (parity_ == parity_even) {
    options.c_cflag &= (tcflag_t) ~(PARODD);
    options.c_cflag |=  (PARENB);
  } else if (parity_ == parity_odd) {
    options.c_cflag |=  (PARENB | PARODD);
  }
#ifdef CMSPAR
  else if (parity_ == parity_mark) {
    options.c_cflag |=  (PARENB | CMSPAR | PARODD);
  }
  else if (parity_ == parity_space) {
    options.c_cflag |=  (PARENB | CMSPAR);
    options.c_cflag &= (tcflag_t) ~(PARODD);
  }
#else
  // CMSPAR is not defined on OSX. So do not support mark or space parity.
  else if (parity_ == parity_mark || parity_ == parity_space) {
    LOG_DEBUG("OS does not support mark or space parity");
  }
#endif  // ifdef CMSPAR
  else {
    LOG_DEBUG("invalid parity");
  }
  // setup flow control
  if (flowcontrol_ == flowcontrol_none) {
    xonxoff_ = false;
    rtscts_ = false;
  }
  if (flowcontrol_ == flowcontrol_software) {
    xonxoff_ = true;
    rtscts_ = false;
  }
  if (flowcontrol_ == flowcontrol_hardware) {
    xonxoff_ = false;
    rtscts_ = true;
  }
  // xonxoff
#ifdef IXANY
  if (xonxoff_)
    options.c_iflag |=  (IXON | IXOFF); //|IXANY)
  else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
#else
  if (xonxoff_)
    options.c_iflag |=  (IXON | IXOFF);
  else
    options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
#endif
  // rtscts
#ifdef CRTSCTS
  if (rtscts_)
    options.c_cflag |=  (CRTSCTS);
  else
    options.c_cflag &= (unsigned long) ~(CRTSCTS);
#elif defined CNEW_RTSCTS
  if (rtscts_)
    options.c_cflag |=  (CNEW_RTSCTS);
  else
    options.c_cflag &= (unsigned long) ~(CNEW_RTSCTS);
#else
#error "OS Support seems wrong."
#endif

  // http://www.unixwiz.net/techtips/termios-vmin-vtime.html
  // this basically sets the read call up to be a polling read,
  // but we are using select to ensure there is data available
  // to read before each call, so we should never needlessly poll
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  // activate settings
  ::tcsetattr (fd_, TCSANOW, &options);

  // apply custom baud rate, if any
  if (custom_baud == true) {
    // Linux Support
    struct serial_struct ser;

    if (-1 == ioctl (fd_, TIOCGSERIAL, &ser)) {
      LOG_DEBUG("reconfigure serial port iotcl failed");
      return false;
    }

    // set custom divisor
    ser.custom_divisor = ser.baud_base / static_cast<int> (baudrate_);
    // update flags
    ser.flags &= ~ASYNC_SPD_MASK;
    ser.flags |= ASYNC_SPD_CUST;

    if (-1 == ioctl (fd_, TIOCSSERIAL, &ser)) {
      LOG_DEBUG("reconfigure serial port iotcl failed");
      return false;
    }
  }

  // Update byte_time_ based on the new settings.
  uint32_t bit_time_ns = 1e9 / baudrate_;
  byte_time_ns_ = bit_time_ns * (1 + bytesize_ + parity_ + stopbits_);

  // Compensate for the stopbits_one_point_five enum being equal to int 3,
  // and not 1.5.
  if (stopbits_ == stopbits_one_point_five) {
    byte_time_ns_ += ((1.5 - stopbits_one_point_five) * bit_time_ns);
  }
  return true;
}

void
Serial::SerialImpl::close ()
{
  if (is_open_ == true) {
    if (fd_ != -1) {
      int ret;
      ret = ::close (fd_);
      if (ret == 0) {
        fd_ = -1;
      }
    }
    is_open_ = false;
  }
}

bool
Serial::SerialImpl::isOpen () const
{
  return is_open_;
}

size_t
Serial::SerialImpl::available ()
{
  int count = 0;
  if (-1 == ioctl (fd_, TIOCINQ, &count)) {
      LOG_WARN("serial port {} available size is {}", port_, count);
  }
  return static_cast<size_t> (count);
}

bool
Serial::SerialImpl::waitReadable (uint32_t timeout)
{
  // Setup a select call to block for serial data or a timeout
  fd_set readfds;
  FD_ZERO (&readfds);
  FD_SET (fd_, &readfds);
  timespec timeout_ts (timespec_from_ms (timeout));
  int r = pselect (fd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);

  if (r < 0) {
    // Select was interrupted
    if (errno == EINTR) {
      return false;
    }
  }
  // Timeout occurred
  if (r == 0) {
    return false;
  }
  // This shouldn't happen, if r > 0 our fd has to be in the list!
  if (!FD_ISSET (fd_, &readfds)) {
    LOG_WARN("select reports ready to read, but our fd isn't"
           " in the list, this shouldn't happen!");
    return false;
  }
  // Data available to read.
  return true;
}

void
Serial::SerialImpl::waitByteTimes (size_t count)
{
  timespec wait_time = { 0, static_cast<long>(byte_time_ns_ * count)};
  pselect (0, NULL, NULL, NULL, &wait_time, NULL);
}

size_t
Serial::SerialImpl::read (uint8_t *buf, size_t size)
{
  // If the port is not open, throw
  if (!is_open_) {
    LOG_WARN("Serial port isn't open and can't read");
  }
  size_t bytes_read = 0;

  // Calculate total timeout in milliseconds t_c + (t_m * N)
  long total_timeout_ms = timeout_.read_timeout_constant;
  total_timeout_ms += timeout_.read_timeout_multiplier * static_cast<long> (size);
  MillisecondTimer total_timeout(total_timeout_ms);

  // Pre-fill buffer with available bytes
  {
    ssize_t bytes_read_now = ::read (fd_, buf, size);
    if (bytes_read_now > 0) {
      bytes_read = bytes_read_now;
    }
  }

  while (bytes_read < size) {
    int64_t timeout_remaining_ms = total_timeout.remaining();
    if (timeout_remaining_ms <= 0) {
      // Timed out
      break;
    }
    // Timeout for the next select is whichever is less of the remaining
    // total read timeout and the inter-byte timeout.
    uint32_t timeout = std::min(static_cast<uint32_t> (timeout_remaining_ms),
                                timeout_.inter_byte_timeout);
    // Wait for the device to be readable, and then attempt to read.
    if (waitReadable(timeout)) {
      // If it's a fixed-length multi-byte read, insert a wait here so that
      // we can attempt to grab the whole thing in a single IO call. Skip
      // this wait if a non-max inter_byte_timeout is specified.
      if (size > 1 && timeout_.inter_byte_timeout == Timeout::max()) {
        size_t bytes_available = available();
        if (bytes_available + bytes_read < size) {
          waitByteTimes(size - (bytes_available + bytes_read));
        }
      }
      // This should be non-blocking returning only what is available now
      //  Then returning so that select can block again.
      ssize_t bytes_read_now =
        ::read (fd_, buf + bytes_read, size - bytes_read);
      // read should always return some data as select reported it was
      // ready to read when we get to this point.
      if (bytes_read_now < 1) {
        // Disconnected devices, at least on Linux, show the
        // behavior that they are always ready to read immediately
        // but reading returns nothing.
        LOG_DEBUG("device reports readiness to read but "
                               "returned no data (device disconnected?)");
      }
      // Update bytes_read
      bytes_read += static_cast<size_t> (bytes_read_now);
      // If bytes_read == size then we have read everything we need
      if (bytes_read == size) {
        break;
      }
      // If bytes_read < size then we have more to read
      if (bytes_read < size) {
        continue;
      }
      // If bytes_read > size then we have over read, which shouldn't happen
      if (bytes_read > size) {
        LOG_DEBUG("read over read, too many bytes where "
                               "read, this shouldn't happen, might be "
                               "a logical error!");
      }
    }
  }
  return bytes_read;
}

size_t
Serial::SerialImpl::write (const uint8_t *data, size_t length)
{
  if (is_open_ == false) {
    LOG_WARN("Serial port isn't open and can't write");
  }
  fd_set writefds;
  size_t bytes_written = 0;

  // Calculate total timeout in milliseconds t_c + (t_m * N)
  long total_timeout_ms = timeout_.write_timeout_constant;
  total_timeout_ms += timeout_.write_timeout_multiplier * static_cast<long> (length);
  MillisecondTimer total_timeout(total_timeout_ms);

  bool first_iteration = true;
  while (bytes_written < length) {
    int64_t timeout_remaining_ms = total_timeout.remaining();
    // Only consider the timeout if it's not the first iteration of the loop
    // otherwise a timeout of 0 won't be allowed through
    if (!first_iteration && (timeout_remaining_ms <= 0)) {
      // Timed out
      break;
    }
    first_iteration = false;

    timespec timeout(timespec_from_ms(timeout_remaining_ms));

    FD_ZERO (&writefds);
    FD_SET (fd_, &writefds);

    // Do the select
    int r = pselect (fd_ + 1, NULL, &writefds, NULL, &timeout, NULL);

    // Figure out what happened by looking at select's response 'r'
    /** Error **/
    if (r < 0) {
      // Select was interrupted, try again
      if (errno == EINTR) {
        continue;
      }
    }
    /** Timeout **/
    if (r == 0) {
      break;
    }
    /** Port ready to write **/
    if (r > 0) {
      // Make sure our file descriptor is in the ready to write list
      if (FD_ISSET (fd_, &writefds)) {
        // This will write some
        ssize_t bytes_written_now =
          ::write (fd_, data + bytes_written, length - bytes_written);

        // even though pselect returned readiness the call might still be 
        // interrupted. In that case simply retry.
        if (bytes_written_now == -1 && errno == EINTR) {
          continue;
        }

        // write should always return some data as select reported it was
        // ready to write when we get to this point.
        if (bytes_written_now < 1) {
          // Disconnected devices, at least on Linux, show the
          // behavior that they are always ready to write immediately
          // but writing returns nothing.
          std::stringstream strs;
          strs << "device reports readiness to write but "
            "returned no data (device disconnected?)";
          strs << " errno=" << errno;
          strs << " bytes_written_now= " << bytes_written_now;
          strs << " bytes_written=" << bytes_written;
          strs << " length=" << length;
          LOG_DEBUG(strs.str().c_str());
        }
        // Update bytes_written
        bytes_written += static_cast<size_t> (bytes_written_now);
        // If bytes_written == size then we have written everything we need to
        if (bytes_written == length) {
          break;
        }
        // If bytes_written < size then we have more to write
        if (bytes_written < length) {
          continue;
        }
        // If bytes_written > size then we have over written, which shouldn't happen
        if (bytes_written > length) {
          LOG_DEBUG("serial port write over wrote, too many bytes where "
                                 "written, this shouldn't happen, might be "
                                 "a logical error!");
        }
      }
    }
  }
  return bytes_written;
}

void
Serial::SerialImpl::setPort (const string &port)
{
  port_ = port;
}

string
Serial::SerialImpl::getPort () const
{
  return port_;
}

void
Serial::SerialImpl::setTimeout (serial::Timeout &timeout)
{
  timeout_ = timeout;
}

serial::Timeout
Serial::SerialImpl::getTimeout () const
{
  return timeout_;
}

void
Serial::SerialImpl::setBaudrate (unsigned long baudrate)
{
  baudrate_ = baudrate;
}

unsigned long
Serial::SerialImpl::getBaudrate () const
{
  return baudrate_;
}

void
Serial::SerialImpl::setBytesize (serial::bytesize_t bytesize)
{
  bytesize_ = bytesize;
}

serial::bytesize_t
Serial::SerialImpl::getBytesize () const
{
  return bytesize_;
}

void
Serial::SerialImpl::setParity (serial::parity_t parity)
{
  parity_ = parity;
}

serial::parity_t
Serial::SerialImpl::getParity () const
{
  return parity_;
}

void
Serial::SerialImpl::setStopbits (serial::stopbits_t stopbits)
{
  stopbits_ = stopbits;
}

serial::stopbits_t
Serial::SerialImpl::getStopbits () const
{
  return stopbits_;
}

void
Serial::SerialImpl::setFlowcontrol (serial::flowcontrol_t flowcontrol)
{
  flowcontrol_ = flowcontrol;
}

serial::flowcontrol_t
Serial::SerialImpl::getFlowcontrol () const
{
  return flowcontrol_;
}

void
Serial::SerialImpl::flush ()
{
  if (is_open_ == false) {
    LOG_WARN("Serial port isn't open and can't flush");
  }
  tcdrain (fd_);
}

void
Serial::SerialImpl::flushInput ()
{
  if (is_open_ == false) {
    LOG_WARN("Serial port isn't open and can't flushInput");
  }
  tcflush (fd_, TCIFLUSH);
}

void
Serial::SerialImpl::flushOutput ()
{
  if (is_open_ == false) {
    LOG_WARN("Serial port isn't open and can't flushOutput");
  }
  tcflush (fd_, TCOFLUSH);
}

void
Serial::SerialImpl::readLock ()
{
  int result = pthread_mutex_lock(&this->read_mutex);
}

void
Serial::SerialImpl::readUnlock ()
{
  int result = pthread_mutex_unlock(&this->read_mutex);
}

void
Serial::SerialImpl::writeLock ()
{
  int result = pthread_mutex_lock(&this->write_mutex);
}

void
Serial::SerialImpl::writeUnlock ()
{
  int result = pthread_mutex_unlock(&this->write_mutex);
}