#include <mpi.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argCount, char** argList) {
  const int PING_PONG_LIMIT = 10;

  // Initialize MPI environment
  MPI_Init(NULL, NULL);
  // find out rank and size
  int world_rank, world_size;
  MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);
  MPI_Comm_size(MPI_COMM_WORLD, &world_size);

  printf("My rank in the world: %d/%d\n", world_rank, world_size);

  MPI_Finalize();
  return 0;
}
