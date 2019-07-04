/**
 * @file SPQR_gamectrlsocket.hpp
 * Implementation of a socket that is used by the module listening to the gamectrl
 * to communicate with the framework.
 * Communication is established on the lo interface, port 4040 and 4141.
 *
 * @see libgamectrl/UdpComm.h
 *
 * @author Dario Albani, Marco Paolelli
 * @quellochefailcaffe Tiziano Manoni
 */

#ifndef SPQRGAMECTRLSOCKET_HPP
#define SPQRGAMECTRLSOCKET_HPP

#define SPQRGAMECTRL_DATA_PORT 4040
#define SPQRGAMECTRL_RETURN_PORT 4141

struct sockaddr;
struct sockaddr_in;

class SPQRgamectrlsocket {
private:
  struct sockaddr* target;
  int socketfd;
  bool resolve(const char*, int, struct sockaddr_in*);

public:
  SPQRgamectrlsocket();
  ~SPQRgamectrlsocket();

   /**
   * Set default target address.
   * @param ip The ip address of the host system.
   * @param port The port used for the connection.
   * \return Does a connection exist?
   */
  bool setTarget(const char* ip, int port);

  /**
   * bind to IN_ADDR_ANY to receive packets
   */
  bool bind(const char* addr, int port);

  /**
   * The function tries to read a package from a socket.
   * @return Number of bytes received or -1 in case of an error.
   */
  int read(char* data, int len);

  /**
   * The function writes a package to a socket.
   * @return True if the package was written.
   */
  bool write(const char* data, const int len);

};

#endif /* SPQRGAMECTRLSOCKET_HPP */
