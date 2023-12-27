#include "UnixSocket.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

UnixSocketClient::UnixSocketClient(std::string path) {
    sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, path.c_str());
    fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
}
UnixSocketClient::~UnixSocketClient() {
    close(sock);
}
int UnixSocketClient::Sendto(const std::string &buf, int length) {
    return sendto(sock, buf.c_str(), length, 0, (sockaddr*)&addr, sizeof(addr));
}

UnixSocketServer::UnixSocketServer(std::string path) {
    sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    addr.sun_family = AF_UNIX;
    strcpy(addr.sun_path, path.c_str());
    unlink(addr.sun_path);
    bind(sock, (sockaddr*)&addr, sizeof(addr));
}
UnixSocketServer::~UnixSocketServer() {
    close(sock);
}
int UnixSocketServer::Recv(std::string &msg, int timeout) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(sock, &fds);
    struct timeval tv;
    tv.tv_sec = timeout / 1000000;
    tv.tv_usec = timeout % 1000000;
    int ret = select(sock + 1, &fds, NULL, NULL, &tv);

    if(!FD_ISSET(sock, &fds)) {
        return 0;
    }

    char buf[1024];
    int n = recv(sock, buf, sizeof(buf), 0);
    msg = std::string(buf, n);
    return n;
}