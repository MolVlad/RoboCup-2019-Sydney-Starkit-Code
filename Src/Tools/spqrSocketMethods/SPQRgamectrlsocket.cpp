/**
 * @file SPQRgamectrlsocket.cpp
 *
 * @see SPQRgamectrlsocket
 * @author Dario Albani, Marco Paolelli e sempre quello che fa il caffe
 */

#include "SPQRgamectrlsocket.hpp"

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <cassert>
#include <cerrno>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <cstring>
#include <net/if.h>
#include <ifaddrs.h>
#include <string>

SPQRgamectrlsocket::SPQRgamectrlsocket() {
  try {
    // create socket file descriptor: use ipv4, dataram format, udp
    socketfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    // set socket options
    int opt = 1;
    setsockopt(socketfd, IPPROTO_IP, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(char));
    // set non blocking
    fcntl(socketfd, F_SETFL, O_NONBLOCK);

    target = (struct sockaddr*)(new struct sockaddr_in);
    assert(socketfd != -1);
  } catch (...) {
    std::cerr << " SPQRgamectrlsocket failed to initialize" << std::endl;
  }
}

SPQRgamectrlsocket::~SPQRgamectrlsocket(){
  close(socketfd);
  delete (struct sockaddr_in*) target;
}

bool SPQRgamectrlsocket::resolve(const char* addrStr, int port, struct sockaddr_in* addr) {
  memset(addr, 0, sizeof(struct sockaddr_in));
  addr->sin_family = AF_INET;
  addr->sin_port = htons(static_cast<unsigned short>(port));

  if(inet_pton(AF_INET, addrStr, &(addr->sin_addr.s_addr)) <= 0) {
    std::cerr << "SPQRgamectrlsocket " << addrStr << " is not a valid dotted ipv4 address" << std::endl;
    return false;
  }

  return true;
}

bool SPQRgamectrlsocket::setTarget(const char* addrStr, int port) {
  struct sockaddr_in* addr = (struct sockaddr_in*) target;
  return resolve(addrStr, port, addr);
}

bool SPQRgamectrlsocket::bind(const char* addr_str, int port) {
  struct sockaddr_in addr;

  // set up socket vars
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(static_cast<unsigned short>(port));
  addr.sin_family = AF_INET;

  // check again if the address is valid
  if(inet_pton(AF_INET, addr_str, &(addr.sin_addr)) <= 0) {
    std::cerr << "SPQRgamectrlsocket::bind() failed: invalid address " << addr_str << std::endl;
    return false;
  }

  if(-1 == ::bind(socketfd, (struct sockaddr*)&addr, sizeof(struct sockaddr_in))) {
    std::cerr << "SPQRgamectrlsocket::bind() failed: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

int SPQRgamectrlsocket::read(char* data, int len) {
  return ::recv(socketfd, data, len, 0);
}

bool SPQRgamectrlsocket::write(const char* data, const int len) {
  return ::sendto(socketfd, data, len, 0, target, sizeof(struct sockaddr_in)) == len;
}
