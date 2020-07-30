#include <cassert>
#include <cstdio>

#define N 3

__global__ void inc(int *a) {
  int i = blockIdx.x;
  if (i < N) {
    a[i]++;
  }
#ifdef __CUDA_ARCH__
  printf ("Hello World!  From device b.t %d.%d\n", i, threadIdx.x);
#endif
}

int main() {
  int ha[N], *da;
  cudaMalloc((void **)&da, N*sizeof(int));
  for (int i = 0; i < N; ++i) {
    ha[i] = i;
  }
  cudaMemcpy(da, ha, N*sizeof(int), cudaMemcpyHostToDevice);
  inc <<<N, 1>>>(da);
  cudaMemcpy(ha, da, N*sizeof(int), cudaMemcpyDeviceToHost);
  for (int i = 0; i < N; ++i) {
    assert(ha[i] == i + 1);
  }
  cudaFree(da);
  printf("All asserts pass - looks like cuda is working!\n");
  return 0;
}
