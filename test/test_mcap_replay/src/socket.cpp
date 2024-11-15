#include <iostream>
#include <cstring>
#include "socket.hpp"

Socket::Socket(std::string server_ip, uint16_t port) {
	if ((_fileDescriptor = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		exit(1);
	}

	memset((char *)&_localAddr, 0, sizeof(_localAddr));
	_localAddr.sin_family = AF_INET;
	_localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	inet_pton(AF_INET, server_ip.c_str(), &_localAddr.sin_addr);
	_localAddr.sin_port = htons(port);
}

bool Socket::send(const std::string& serialized, bool returning) {
	return sendto(_fileDescriptor, serialized.data(), serialized.size(), 0, 
		(struct sockaddr*)& (returning ? _remAddr: _localAddr), 
		sizeof(_localAddr)) != -1;
}

bool Socket::receive(std::string& serialized) {
	const int bufferSize = 1024;
    socklen_t addrlen = sizeof(_remAddr);
    unsigned char buf[bufferSize];  

    int recvlen = recvfrom(_fileDescriptor, buf, bufferSize, 0, (struct sockaddr *)&_remAddr, &addrlen);
    if (recvlen < 0) {
        return false;
    } else {
        // std::cout << "SOCKET DEBUG - recvlen: "<< recvlen << std::endl;
		serialized.assign(reinterpret_cast<char*>(buf), recvlen);
		return true;
    }
}


void Socket::close() {
	::close(_fileDescriptor);
}

bool Socket::bind() {
	return ::bind(_fileDescriptor, (struct sockaddr*)& _localAddr, sizeof(_localAddr)) >= 0;
}
