#include "helpers.h"
#include <stdlib.h>
#include <stdio.h>

char* allocate(int mb_count) {
  size_t size = (1 << 20) * mb_count;
  printf("  mb_count: %d\n", mb_count);
  printf("  allocating %Ld bytes\n", size);
  if (size < 0) {
    exit(-1);
  }
  char* buffer = malloc(sizeof(char) * size);
  printf("  buffer pointer: 0x%08x\n", buffer);
  return buffer;
}

void fill(char* buffer, int mb_count) {
  for (size_t i = 0; i < (1 << 20) * mb_count; i++) {
    buffer[i] = 0;
  }
}
