#include "UDPServer.h"

#include "Logger.h"
// using namespace Lidars;

/** \brief Constructor
 * \param[in] udp receive port
 */
UDPServer::UDPServer(int port) {
  socket = nullptr;

  namespace ip = boost::asio::ip;
  socket.reset(new ip::udp::socket(io_service, ip::udp::v4()));
  socket->set_option(boost::asio::socket_base::reuse_address(true));
  socket->set_option(boost::asio::socket_base::do_not_route(true));
  socket->set_option(boost::asio::socket_base::receive_buffer_size(32*1024*1024));
  socket->set_option(boost::asio::socket_base::send_buffer_size(32*1024*1024));
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

int UDPServer::UDPSendto(const std::string &addr, int port, const std::string &buf,
                         int length) {
  int size = 0;
  try {
    boost::asio::ip::udp::endpoint destination(
        boost::asio::ip::address::from_string(addr), port);
    size = socket->send_to(boost::asio::buffer(buf.c_str(), length), destination);
  } catch(std::exception const& ex) {
    LOG_ERROR(ex.what());
  }
  return size;
}

int UDPServer::UDPSendtoBuf(const std::string &addr, int port, char* buf,
                         int length) {
  int size = 0;
  try {
    boost::asio::ip::udp::endpoint destination(
        boost::asio::ip::address::from_string(addr), port);
    size = socket->send_to(boost::asio::buffer(buf, length), destination);
  }  catch(std::exception const& ex) {
    LOG_ERROR(ex.what());
  }
  return size;
}

/** \brief get sender's address
 * \return sender's address (std::string)
 */
std::string UDPServer::getSenderAddr(void) {
  return sender_endpoint.address().to_string();
}

int UDPServer::getSenderPort(void) { return sender_endpoint.port(); }
