#include "helpers.h"

#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argCount, char *argList[]) {
  int mb_count = 1; // number of megabytes to allocate
  if (argCount > 1) {
    if (strcmp(argList[1], "--help") == 0 || strcmp(argList[1], "-h") == 0) {
      printf("Usage: alloc <num-megabytes>\n");
      return 0;
    }
    mb_count = atoi(argList[1]);
  }

  printf("Allocating %d MB\n", mb_count);

  char* buffer = allocate(mb_count);

  printf("Filling allocation with all zeros\n");
  fill(buffer, mb_count);

  int sleep_secs = 5; // seconds
  printf("Sleeping for %d seconds\n", sleep_secs);
  sleep(sleep_secs);

  printf("Done!\n");

  return 0;
  }
