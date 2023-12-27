//============================================================================================================
//!
//! \file UDPServer.h
//!
//! \brief UDPServer operation file.
//!
//============================================================================================================
#ifndef __UDPSERVER_H
#define __UDPSERVER_H

#include <iostream>
#include <memory>

#include <boost/asio.hpp>

class UDPServer {
 public:
  /** \brief Constructor
   * \param[in] udp receive port
   */
  UDPServer(int port = 0);
  ~UDPServer();

  /** \brief UDP server receive
   * \param[in] receive buffer
   * \param[in] buffer length
   */
  int UDPServerReceive(char buf[], int length);

  int UDPSendto(std::string addr, int port, std::string buf, int length);
  int UDPSendtoBuf(std::string addr, int port, char* buf, int length);
  /** \brief get sender's address
   * \return sender's address (std::string)
   */
  std::string getSenderAddr(void);

  /** \brief get sender's port
   * \return sender's port (int)
   */
  int getSenderPort(void);

 private:
  boost::asio::io_service io_service;
  std::unique_ptr<boost::asio::ip::udp::socket> socket;
  boost::asio::ip::udp::endpoint sender_endpoint;
};

#endif  //__UDPSERVER_H
