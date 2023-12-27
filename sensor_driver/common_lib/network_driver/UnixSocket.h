#ifndef __UNIX_SOCKET_H
#define __UNIX_SOCKET_H

#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

class UnixSocketClient {
 public:

  UnixSocketClient(std::string path);
  ~UnixSocketClient();
  int Sendto(const std::string &buf, int length);

 private:
  int sock;
  sockaddr_un addr;
  socklen_t addrlen;
};

class UnixSocketServer {
 public:

  UnixSocketServer(std::string path);
  ~UnixSocketServer();
  int Recv(std::string &msg, int timeout = 100000);

 private:
  int sock;
  sockaddr_un addr;
  socklen_t addrlen;
};

#endif  //__UNIX_SOCKET_H
