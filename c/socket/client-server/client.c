#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#define PORT 9433
#define UNUSED(x) (void)(x)

int main(int arg_count, char *arg_list[]) {
  UNUSED(arg_count);
  UNUSED(arg_list);
  int obj_socket = 0;
  int reader;
  struct sockaddr_in serv_addr;
  char *message = "A message from client !";
  char buffer[1024] = {0};
  if ((obj_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
    printf("Socket creation error");
    return -1;
  }
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORT);
  // Converting IPv4 and IPv6 addresses from text to binary form
  if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
    printf("\nInvalid address.  This IP address is not supported\n");
    return -1;
  }
  if (connect(obj_socket, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    printf("Connection failed: can't establish a connection of this socket");
    return -1;
  }
  send(obj_socket, message, strlen(message), 0);
  printf("\nClient: message has been sent\n");
  reader = read(obj_socket, buffer, 1024);
  UNUSED(reader);
  printf("%s\n", buffer);
  return 0;
}
