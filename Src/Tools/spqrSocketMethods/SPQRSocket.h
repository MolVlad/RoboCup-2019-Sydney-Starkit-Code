#include <sys/un.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdlib.h>

#include <stddef.h>
#include <stdio.h>
#include <errno.h>
#include <iostream>

extern "C"{

int make_named_socket (const char *filename){
  struct sockaddr_un name;
  int sock;
  size_t size;

  /* Create the socket. */
  sock = socket (AF_UNIX, SOCK_STREAM, 0);
  if (sock < 0)
    {
      //perror ("socket");
      exit (EXIT_FAILURE);
    }

  /* Bind a name to the socket. */
  name.sun_family = AF_LOCAL;
  strncpy (name.sun_path, filename, sizeof (name.sun_path));
  name.sun_path[sizeof (name.sun_path) - 1] = '\0';

  /* The size of the address is
     the offset of the start of the filename,
     plus its length (not including the terminating null byte).
     Alternatively you can just do:
     size = SUN_LEN (&name);
 */
  size = (offsetof (struct sockaddr_un, sun_path)
          + strlen (name.sun_path));

  //if (bind (sock, (struct sockaddr *) &name, size) < 0)
  if(connect(sock, (struct sockaddr *) &name, size ) < 0)
    {
      //perror ("Marcello ");
      //std::cout<<"entrato in errore "<<std::endl;
      exit (EXIT_FAILURE);
    }

  return sock;
}	

int ricevi(int socket, char *buff, int size){
    int byteRead = 0;
    int ret = 0;
    while(byteRead < size){
        
      ret = recv(socket, buff + byteRead, size - byteRead, 0);  

      if(ret == -1 && errno == EINTR)
        continue;
      
      if(ret <= 0 ){
        //perror("Error: the socket has been closed");
        return -1;
      }

      byteRead += ret;
    }
    return byteRead;
  }

  int invia(int socket, char *buff, int size){
    int byteSend = 0;
    int ret = 0;

    while(byteSend < size){

      ret = send(socket, buff + byteSend, size - byteSend, 0);  
      
      if(ret == -1 && errno == EINTR)
        continue;
      
      if(ret <= 0 ){
        //perror("Error: the socket has been closed");
        return -1;
      }

      byteSend += ret;

    }
}

}
