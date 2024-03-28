#ifndef __SERIAL_DRIVER_H
#define __SERIAL_DRIVER_H

#include <limits>
#include <vector>
#include <string>
#include <cstring>
#include <sstream>

#include <stddef.h>
#include <stdio.h>
#include <stdint.h>

#include <pthread.h>

using std::size_t;
using std::string;
using std::invalid_argument;

namespace serial {

/*!
 * Enumeration defines the possible bytesizes for the serial port.
 */
typedef enum {
  fivebits = 5,
  sixbits = 6,
  sevenbits = 7,
  eightbits = 8
} bytesize_t;

/*!
 * Enumeration defines the possible parity types for the serial port.
 */
typedef enum {
  parity_none = 0,
  parity_odd = 1,
  parity_even = 2,
  parity_mark = 3,
  parity_space = 4
} parity_t;

/*!
 * Enumeration defines the possible stopbit types for the serial port.
 */
typedef enum {
  stopbits_one = 1,
  stopbits_two = 2,
  stopbits_one_point_five
} stopbits_t;

/*!
 * Enumeration defines the possible flowcontrol types for the serial port.
 */
typedef enum {
  flowcontrol_none = 0,
  flowcontrol_software,
  flowcontrol_hardware
} flowcontrol_t;

/*!
 * Structure for setting the timeout of the serial port, times are
 * in milliseconds.
 *
 * In order to disable the interbyte timeout, set it to Timeout::max().
 */
struct Timeout {
#ifdef max
# undef max
#endif
  static uint32_t max() {return std::numeric_limits<uint32_t>::max();}
  /*!
   * Convenience function to generate Timeout structs using a
   * single absolute timeout.
   *
   * \param timeout A long that defines the time in milliseconds until a
   * timeout occurs after a call to read or write is made.
   *
   * \return Timeout struct that represents this simple timeout provided.
   */
  static Timeout simpleTimeout(uint32_t timeout) {
    return Timeout(max(), timeout, 0, timeout, 0);
  }

  /*! Number of milliseconds between bytes received to timeout on. */
  uint32_t inter_byte_timeout;
  /*! A constant number of milliseconds to wait after calling read. */
  uint32_t read_timeout_constant;
  /*! A multiplier against the number of requested bytes to wait after
   *  calling read.
   */
  uint32_t read_timeout_multiplier;
  /*! A constant number of milliseconds to wait after calling write. */
  uint32_t write_timeout_constant;
  /*! A multiplier against the number of requested bytes to wait after
   *  calling write.
   */
  uint32_t write_timeout_multiplier;

  explicit Timeout (uint32_t inter_byte_timeout_=0,
                    uint32_t read_timeout_constant_=0,
                    uint32_t read_timeout_multiplier_=0,
                    uint32_t write_timeout_constant_=0,
                    uint32_t write_timeout_multiplier_=0)
  : inter_byte_timeout(inter_byte_timeout_),
    read_timeout_constant(read_timeout_constant_),
    read_timeout_multiplier(read_timeout_multiplier_),
    write_timeout_constant(write_timeout_constant_),
    write_timeout_multiplier(write_timeout_multiplier_)
  {}
};

/*!
 * Class that provides a portable serial port interface.
 */
class Serial {
public:
  /*!
   * Creates a Serial object and opens the port if a port is specified,
   * otherwise it remains closed until serial::Serial::open is called.
   *
   * \param port A std::string containing the address of the serial port,
   *        which would be something like 'COM1' on Windows and '/dev/ttyS0'
   *        on Linux.
   *
   * \param baudrate An unsigned 32-bit integer that represents the baudrate
   *
   * \param timeout A serial::Timeout struct that defines the timeout
   * conditions for the serial port. \see serial::Timeout
   *
   * \param bytesize Size of each byte in the serial transmission of data,
   * default is eightbits, possible values are: fivebits, sixbits, sevenbits,
   * eightbits
   *
   * \param parity Method of parity, default is parity_none, possible values
   * are: parity_none, parity_odd, parity_even
   *
   * \param stopbits Number of stop bits used, default is stopbits_one,
   * possible values are: stopbits_one, stopbits_one_point_five, stopbits_two
   *
   * \param flowcontrol Type of flowcontrol used, default is
   * flowcontrol_none, possible values are: flowcontrol_none,
   * flowcontrol_software, flowcontrol_hardware
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::IOException
   * \throw std::invalid_argument
   */
  Serial (const std::string &port = "",
          uint32_t baudrate = 9600,
          Timeout timeout = Timeout(),
          bytesize_t bytesize = eightbits,
          parity_t parity = parity_none,
          stopbits_t stopbits = stopbits_one,
          flowcontrol_t flowcontrol = flowcontrol_none);

  /*! Destructor */
  virtual ~Serial ();

  /*!
   * Opens the serial port as long as the port is set and the port isn't
   * already open.
   *
   * If the port is provided to the constructor then an explicit call to open
   * is not needed.
   *
   * \see Serial::Serial
   *
   * \throw std::invalid_argument
   * \throw serial::SerialException
   * \throw serial::IOException
   */
  void
  open ();

  /*! Gets the open status of the serial port.
   *
   * \return Returns true if the port is open, false otherwise.
   */
  bool
  isOpen () const;

  /*! Closes the serial port. */
  void
  close ();

  /*! Return the number of characters in the buffer. */
  size_t
  available ();

  /*! Block until there is serial data to read or read_timeout_constant
   * number of milliseconds have elapsed. The return value is true when
   * the function exits with the port in a readable state, false otherwise
   * (due to timeout or select interruption). */
  bool
  waitReadable ();

  /*! Block for a period of time corresponding to the transmission time of
   * count characters at present serial settings. This may be used in con-
   * junction with waitReadable to read larger blocks of data from the
   * port. */
  void
  waitByteTimes (size_t count);

  /*! Read a given amount of bytes from the serial port into a given buffer.
   *
   * The read function will return in one of three cases:
   *  * The number of requested bytes was read.
   *    * In this case the number of bytes requested will match the size_t
   *      returned by read.
   *  * A timeout occurred, in this case the number of bytes read will not
   *    match the amount requested, but no exception will be thrown.  One of
   *    two possible timeouts occurred:
   *    * The inter byte timeout expired, this means that number of
   *      milliseconds elapsed between receiving bytes from the serial port
   *      exceeded the inter byte timeout.
   *    * The total timeout expired, which is calculated by multiplying the
   *      read timeout multiplier by the number of requested bytes and then
   *      added to the read timeout constant.  If that total number of
   *      milliseconds elapses after the initial call to read a timeout will
   *      occur.
   *  * An exception occurred, in this case an actual exception will be thrown.
   *
   * \param buffer An uint8_t array of at least the requested size.
   * \param size A size_t defining how many bytes to be read.
   *
   * \return A size_t representing the number of bytes read as a result of the
   *         call to read.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  size_t
  read (uint8_t *buffer, size_t size);

  /*! Read a given amount of bytes from the serial port into a give buffer.
   *
   * \param buffer A reference to a std::vector of uint8_t.
   * \param size A size_t defining how many bytes to be read.
   *
   * \return A size_t representing the number of bytes read as a result of the
   *         call to read.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  size_t
  read (std::vector<uint8_t> &buffer, size_t size = 1);

  /*! Read a given amount of bytes from the serial port into a give buffer.
   *
   * \param buffer A reference to a std::string.
   * \param size A size_t defining how many bytes to be read.
   *
   * \return A size_t representing the number of bytes read as a result of the
   *         call to read.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  size_t
  read (std::string &buffer, size_t size = 1);

  /*! Read a given amount of bytes from the serial port and return a string
   *  containing the data.
   *
   * \param size A size_t defining how many bytes to be read.
   *
   * \return A std::string containing the data read from the port.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  std::string
  read (size_t size = 1);

  /*! Reads in a line or until a given delimiter has been processed.
   *
   * Reads from the serial port until a single line has been read.
   *
   * \param buffer A std::string reference used to store the data.
   * \param size A maximum length of a line, defaults to 65536 (2^16)
   * \param eol A string to match against for the EOL.
   *
   * \return A size_t representing the number of bytes read.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  size_t
  readline (std::string &buffer, size_t size = 65536, std::string eol = "\n");

  /*! Reads in a line or until a given delimiter has been processed.
   *
   * Reads from the serial port until a single line has been read.
   *
   * \param size A maximum length of a line, defaults to 65536 (2^16)
   * \param eol A string to match against for the EOL.
   *
   * \return A std::string containing the line.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  std::string
  readline (size_t size = 65536, std::string eol = "\n");

  /*! Reads in multiple lines until the serial port times out.
   *
   * This requires a timeout > 0 before it can be run. It will read until a
   * timeout occurs and return a list of strings.
   *
   * \param size A maximum length of combined lines, defaults to 65536 (2^16)
   *
   * \param eol A string to match against for the EOL.
   *
   * \return A vector<string> containing the lines.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   */
  std::vector<std::string>
  readlines (size_t size = 65536, std::string eol = "\n");

  /*! Write a string to the serial port.
   *
   * \param data A const reference containing the data to be written
   * to the serial port.
   *
   * \param size A size_t that indicates how many bytes should be written from
   * the given data buffer.
   *
   * \return A size_t representing the number of bytes actually written to
   * the serial port.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   * \throw serial::IOException
   */
  size_t
  write (const uint8_t *data, size_t size);

  /*! Write a string to the serial port.
   *
   * \param data A const reference containing the data to be written
   * to the serial port.
   *
   * \return A size_t representing the number of bytes actually written to
   * the serial port.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   * \throw serial::IOException
   */
  size_t
  write (const std::vector<uint8_t> &data);

  /*! Write a string to the serial port.
   *
   * \param data A const reference containing the data to be written
   * to the serial port.
   *
   * \return A size_t representing the number of bytes actually written to
   * the serial port.
   *
   * \throw serial::PortNotOpenedException
   * \throw serial::SerialException
   * \throw serial::IOException
   */
  size_t
  write (const std::string &data);

  /*! Sets the serial port identifier.
   *
   * \param port A const std::string reference containing the address of the
   * serial port, which would be something like 'COM1' on Windows and
   * '/dev/ttyS0' on Linux.
   *
   * \throw std::invalid_argument
   */
  void
  setPort (const std::string &port);

  /*! Gets the serial port identifier.
   *
   * \see Serial::setPort
   *
   * \throw std::invalid_argument
   */
  std::string
  getPort () const;

  /*! Sets the timeout for reads and writes using the Timeout struct.
   *
   * There are two timeout conditions described here:
   *  * The inter byte timeout:
   *    * The inter_byte_timeout component of serial::Timeout defines the
   *      maximum amount of time, in milliseconds, between receiving bytes on
   *      the serial port that can pass before a timeout occurs.  Setting this
   *      to zero will prevent inter byte timeouts from occurring.
   *  * Total time timeout:
   *    * The constant and multiplier component of this timeout condition,
   *      for both read and write, are defined in serial::Timeout.  This
   *      timeout occurs if the total time since the read or write call was
   *      made exceeds the specified time in milliseconds.
   *    * The limit is defined by multiplying the multiplier component by the
   *      number of requested bytes and adding that product to the constant
   *      component.  In this way if you want a read call, for example, to
   *      timeout after exactly one second regardless of the number of bytes
   *      you asked for then set the read_timeout_constant component of
   *      serial::Timeout to 1000 and the read_timeout_multiplier to zero.
   *      This timeout condition can be used in conjunction with the inter
   *      byte timeout condition with out any problems, timeout will simply
   *      occur when one of the two timeout conditions is met.  This allows
   *      users to have maximum control over the trade-off between
   *      responsiveness and efficiency.
   *
   * Read and write functions will return in one of three cases.  When the
   * reading or writing is complete, when a timeout occurs, or when an
   * exception occurs.
   *
   * A timeout of 0 enables non-blocking mode.
   *
   * \param timeout A serial::Timeout struct containing the inter byte
   * timeout, and the read and write timeout constants and multipliers.
   *
   * \see serial::Timeout
   */
  void
  setTimeout (Timeout &timeout);

  /*! Sets the timeout for reads and writes. */
  void
  setTimeout (uint32_t inter_byte_timeout, uint32_t read_timeout_constant,
              uint32_t read_timeout_multiplier, uint32_t write_timeout_constant,
              uint32_t write_timeout_multiplier)
  {
    Timeout timeout(inter_byte_timeout, read_timeout_constant,
                    read_timeout_multiplier, write_timeout_constant,
                    write_timeout_multiplier);
    return setTimeout(timeout);
  }

  /*! Gets the timeout for reads in seconds.
   *
   * \return A Timeout struct containing the inter_byte_timeout, and read
   * and write timeout constants and multipliers.
   *
   * \see Serial::setTimeout
   */
  Timeout
  getTimeout () const;

  /*! Sets the baudrate for the serial port.
   *
   * Possible baudrates depends on the system but some safe baudrates include:
   * 110, 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 56000,
   * 57600, 115200
   * Some other baudrates that are supported by some comports:
   * 128000, 153600, 230400, 256000, 460800, 500000, 921600
   *
   * \param baudrate An integer that sets the baud rate for the serial port.
   *
   * \throw std::invalid_argument
   */
  void
  setBaudrate (uint32_t baudrate);

  /*! Gets the baudrate for the serial port.
   *
   * \return An integer that sets the baud rate for the serial port.
   *
   * \see Serial::setBaudrate
   *
   * \throw std::invalid_argument
   */
  uint32_t
  getBaudrate () const;

  /*! Sets the bytesize for the serial port.
   *
   * \param bytesize Size of each byte in the serial transmission of data,
   * default is eightbits, possible values are: fivebits, sixbits, sevenbits,
   * eightbits
   *
   * \throw std::invalid_argument
   */
  void
  setBytesize (bytesize_t bytesize);

  /*! Gets the bytesize for the serial port.
   *
   * \see Serial::setBytesize
   *
   * \throw std::invalid_argument
   */
  bytesize_t
  getBytesize () const;

  /*! Sets the parity for the serial port.
   *
   * \param parity Method of parity, default is parity_none, possible values
   * are: parity_none, parity_odd, parity_even
   *
   * \throw std::invalid_argument
   */
  void
  setParity (parity_t parity);

  /*! Gets the parity for the serial port.
   *
   * \see Serial::setParity
   *
   * \throw std::invalid_argument
   */
  parity_t
  getParity () const;

  /*! Sets the stopbits for the serial port.
   *
   * \param stopbits Number of stop bits used, default is stopbits_one,
   * possible values are: stopbits_one, stopbits_one_point_five, stopbits_two
   *
   * \throw std::invalid_argument
   */
  void
  setStopbits (stopbits_t stopbits);

  /*! Gets the stopbits for the serial port.
   *
   * \see Serial::setStopbits
   *
   * \throw std::invalid_argument
   */
  stopbits_t
  getStopbits () const;

  /*! Sets the flow control for the serial port.
   *
   * \param flowcontrol Type of flowcontrol used, default is flowcontrol_none,
   * possible values are: flowcontrol_none, flowcontrol_software,
   * flowcontrol_hardware
   *
   * \throw std::invalid_argument
   */
  void
  setFlowcontrol (flowcontrol_t flowcontrol);

  /*! Gets the flow control for the serial port.
   *
   * \see Serial::setFlowcontrol
   *
   * \throw std::invalid_argument
   */
  flowcontrol_t
  getFlowcontrol () const;

  /*! Flush the input and output buffers */
  void
  flush ();

  /*! Flush only the input buffer */
  void
  flushInput ();

  /*! Flush only the output buffer */
  void
  flushOutput ();

private:
  // Disable copy constructors
  Serial(const Serial&);
  Serial& operator=(const Serial&);

  // Pimpl idiom, d_pointer
  class SerialImpl;
  SerialImpl *pimpl_;

  // Scoped Lock Classes
  class ScopedReadLock;
  class ScopedWriteLock;

  // Read common function
  size_t
  read_ (uint8_t *buffer, size_t size);
  // Write common function
  size_t
  write_ (const uint8_t *data, size_t length);

};

class MillisecondTimer {
public:
  MillisecondTimer(const uint32_t millis);         
  int64_t remaining();

private:
  static timespec timespec_now();
  timespec expiry;
};

class serial::Serial::SerialImpl {
public:
  SerialImpl (const string &port,
              unsigned long baudrate,
              bytesize_t bytesize,
              parity_t parity,
              stopbits_t stopbits,
              flowcontrol_t flowcontrol);

  virtual ~SerialImpl ();

  void
  open ();

  void
  close ();

  bool
  isOpen () const;

  size_t
  available ();

  bool
  waitReadable (uint32_t timeout);

  void
  waitByteTimes (size_t count);

  size_t
  read (uint8_t *buf, size_t size = 1);

  size_t
  write (const uint8_t *data, size_t length);

  void
  flush ();

  void
  flushInput ();

  void
  flushOutput ();

  void
  setPort (const string &port);

  string
  getPort () const;

  void
  setTimeout (Timeout &timeout);

  Timeout
  getTimeout () const;

  void
  setBaudrate (unsigned long baudrate);

  unsigned long
  getBaudrate () const;

  void
  setBytesize (bytesize_t bytesize);

  bytesize_t
  getBytesize () const;

  void
  setParity (parity_t parity);

  parity_t
  getParity () const;

  void
  setStopbits (stopbits_t stopbits);

  stopbits_t
  getStopbits () const;

  void
  setFlowcontrol (flowcontrol_t flowcontrol);

  flowcontrol_t
  getFlowcontrol () const;

  void
  readLock ();

  void
  readUnlock ();

  void
  writeLock ();

  void
  writeUnlock ();

protected:
  bool reconfigurePort ();

private:
  string port_;               // Path to the file descriptor
  int fd_;                    // The current file descriptor

  bool is_open_;
  bool xonxoff_;
  bool rtscts_;

  Timeout timeout_;           // Timeout for read operations
  unsigned long baudrate_;    // Baudrate
  uint32_t byte_time_ns_;     // Nanoseconds to transmit/receive a single byte

  parity_t parity_;           // Parity
  bytesize_t bytesize_;       // Size of the bytes
  stopbits_t stopbits_;       // Stop Bits
  flowcontrol_t flowcontrol_; // Flow Control

  // Mutex used to lock the read functions
  pthread_mutex_t read_mutex;
  // Mutex used to lock the write functions
  pthread_mutex_t write_mutex;
};

}
#endif // __SERIAL_DRIVER_H