#pragma once
#include "udpm.hpp"
#include "buffers.hpp"

class UDPMAddress
{
  public:
    UDPMAddress(const string& ip, u16 port)
    {
        this->ip = ip;
        this->port = port;

        memset(&this->addr, 0, sizeof(this->addr));
        this->addr.sin_family = AF_INET;
        inet_aton(ip.c_str(), &this->addr.sin_addr);
        this->addr.sin_port = htons(port);
    }

    const string& getIP() const { return ip; }
    u16 getPort() const { return port; }
    struct sockaddr* getAddrPtr() const { return (struct sockaddr*)&addr; }
    size_t getAddrSize() const { return sizeof(addr); }

  private:
    string ip;
    u16 port;
    struct sockaddr_in addr;
};

class UDPMSocket
{
  public:
    UDPMSocket();
    ~UDPMSocket();
    bool isOpen();
    void close();

    bool init();
    bool joinMulticastGroup(struct in_addr multiaddr);
    bool setTTL(u8 ttl);
    bool bindPort(u16 port);
    bool setReuseAddr();
    bool setReusePort();
    bool enablePacketTimestamp();
    bool enableLoopback();
    bool setDestination(const string& ip, u16 port);

    size_t getRecvBufSize();
    size_t getSendBufSize();

    // Returns true when there is a packet available for receiving
    bool waitUntilData(int timeout);
    int recvPacket(Packet *pkt);

    ssize_t sendBuffers(const UDPMAddress& dest, const char *a, size_t alen);
    ssize_t sendBuffers(const UDPMAddress& dest, const char *a, size_t alen,
                            const char *b, size_t blen);
    ssize_t sendBuffers(const UDPMAddress& dest, const char *a, size_t alen,
                        const char *b, size_t blen, const char *c, size_t clen);

    static bool checkConnection(const string& ip, u16 port);
    void checkAndWarnAboutSmallBuffer(size_t datalen, size_t kbufsize);

    static UDPMSocket createSendSocket(struct in_addr multiaddr, u8 ttl);
    static UDPMSocket createRecvSocket(struct in_addr multiaddr, u16 port);

  private:
    SOCKET fd = -1;
    bool warnedAboutSmallBuffer = false;

  private:
    // Disallow copies
    UDPMSocket(const UDPMSocket&) = delete;
    UDPMSocket& operator=(const UDPMSocket&) = delete;

  public:
    // Allow moves
    UDPMSocket(UDPMSocket&& other) { std::swap(this->fd, other.fd); }
    UDPMSocket& operator=(UDPMSocket&& other) { std::swap(this->fd, other.fd); return *this; }
};
