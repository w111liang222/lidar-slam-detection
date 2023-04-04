#include "UDPServer.h"

/** \brief Constructor
 * \param[in] udp receive port
 */
UDPServer::UDPServer(int port) {
  socket = nullptr;

  namespace ip = boost::asio::ip;
  socket.reset(new ip::udp::socket(io_service, ip::udp::v4()));
  socket->set_option(boost::asio::socket_base::reuse_address(true));
  socket->set_option(boost::asio::socket_base::receive_buffer_size(1*1024*1024));
  if (port > 0) {
    socket->bind(ip::udp::endpoint(ip::udp::v4(), port));
  }
}
UDPServer::~UDPServer() {}

/** \brief UDP server receive
 * \param[in] receive buffer
 * \param[in] buffer length
 */
int UDPServer::UDPServerReceive(char buf[], int length) {
  return socket->receive_from(boost::asio::buffer(buf, length),
                              sender_endpoint);
}

int UDPServer::UDPSendto(std::string addr, int port, std::string buf,
                         int length) {
  boost::asio::ip::udp::endpoint destination(
      boost::asio::ip::address::from_string(addr), port);
  return socket->send_to(boost::asio::buffer(buf.c_str(), length), destination);
}

int UDPServer::UDPSendtoBuf(std::string addr, int port, char* buf,
                         int length) {
  boost::asio::ip::udp::endpoint destination(
      boost::asio::ip::address::from_string(addr), port);
  return socket->send_to(boost::asio::buffer(buf, length), destination);
}

/** \brief get sender's address
 * \return sender's address (std::string)
 */
std::string UDPServer::getSenderAddr(void) {
  return sender_endpoint.address().to_string();
}

int UDPServer::getSenderPort(void) { return sender_endpoint.port(); }
