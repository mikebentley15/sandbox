#include <stdio.h>
#include <unistd.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <stdlib.h>

#define PORT 9433
#define UNUSED(x) (void)(x)

int main(int arg_count, char *arg_list[]) {
  UNUSED(arg_count);
  UNUSED(arg_list);
  int obj_server, sock, reader;
  struct sockaddr_in address;
  int opted = 1;
  int address_length = sizeof(address);
  char buffer[1024] = {0};
  char *message = "A message from server !";
  if ((obj_server = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
    perror("Opening of a socket failed");
    exit(EXIT_FAILURE);
  }
  if (setsockopt(obj_server, SOL_SOCKET, SO_REUSEADDR, &opted, sizeof(opted))) {
    perror("Can't set the socket");
    exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(PORT);
  if (bind(obj_server, (struct sockaddr*)&address, sizeof(address)) < 0) {
    perror("Binding of socket failed");
    exit(EXIT_FAILURE);
  }
  if (listen(obj_server, 3) < 0) {
    perror("Can't listen from the server");
    exit(EXIT_FAILURE);
  }
  if ((sock = accept(obj_server, (struct sockaddr*)&address, (socklen_t*)&address_length)) < 0) {
    perror("Accept");
    exit(EXIT_FAILURE);
  }
  reader = read(sock, buffer, 1024);
  UNUSED(reader);
  printf("%s\n", buffer);
  send(sock, message, strlen(message), 0);
  printf("Server: Message has been sent\n");
  return 0;
}
