#include <iostream>
#include <cstring>
#include "socket.hpp"

Socket::Socket(std::string server_ip, uint16_t port) {
	if ((fileDescriptor = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		exit(1);
	}

	memset((char *)&localAddr, 0, sizeof(localAddr));
	localAddr.sin_family = AF_INET;
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	inet_pton(AF_INET, server_ip.c_str(), &localAddr.sin_addr);
	localAddr.sin_port = htons(port);
}

bool Socket::send(const std::string& serialized, bool returning){
	return sendto(fileDescriptor, serialized.data(), serialized.size(), 0, 
		(struct sockaddr*)& (returning ? remAddr: localAddr), 
		sizeof(localAddr)) != -1;
}

bool Socket::receive(std::string& serialized) {
	const int bufferSize = 1024;
    socklen_t addrlen = sizeof(remAddr);
    unsigned char buf[bufferSize];  

    int recvlen = recvfrom(fileDescriptor, buf, bufferSize, 0, (struct sockaddr *)&remAddr, &addrlen);
    if (recvlen < 0) {
        return false;
    } else {
        // std::cout << "SOCKET DEBUG - recvlen: "<< recvlen << std::endl;
		serialized.assign(reinterpret_cast<char*>(buf), recvlen);
		return true;
    }
}


void Socket::close() {
	::close(fileDescriptor);
}

bool Socket::bind() {
	return ::bind(fileDescriptor, (struct sockaddr*)& localAddr, sizeof(localAddr)) >= 0;
}
