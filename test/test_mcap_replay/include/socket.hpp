#ifndef SOCKET_HPP 
#define SOCKET_HPP 

#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdexcept>

static const std::string DEFAULT_IP = "127.0.0.1";
static const uint16_t DEFAULT_PORT = 1153;

class Socket {
private:
    int fileDescriptor;
    struct sockaddr_in localAddr;
    struct sockaddr_in remAddr;

public:
    Socket (std::string server_ip, uint16_t port);

    void close();
    bool send (const std::string&, bool returning);
    bool receive (std::string&);
    bool bind();
};

#endif