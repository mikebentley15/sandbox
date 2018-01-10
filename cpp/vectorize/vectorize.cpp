#include <math.h>
#include <stdlib.h>
#include <stdio.h>

typedef float value_type;
typedef unsigned long index_type;

static value_type F(value_type v) {
  return 1.0/(1.0 + exp(-v));
}

static value_type DF(value_type v) {
  const value_type Ev = exp(-v);
  return Ev/((1.0 + Ev)*(1.0 + Ev));
}

#ifndef WITH_BLAS

static void get_Yp(const value_type * __restrict__ Ap, const value_type * __restrict__ W,
		   value_type * __restrict__ Yp, const value_type * __restrict__ Dp,
		   const index_type height, const index_type width) {
  for (index_type i=0;i<width;i++) {
    Yp[height*width + i] = 2*DF(Ap[height*width + i])*(Dp[i] - F(Ap[height*width + i]));
  }

  for (index_type k=height-1;k+1>0;k--) {
    for (index_type i=0;i<width;i++) {
      double sum = 0.0;
      for (index_type j=0;j<width;j++) {
        sum += W[k*width*width + j*width + i]*Yp[(k+1)*width + j];
      }
      Yp[k*width + i] = sum * DF(Ap[k*width + i]);
    }
  }
}

#else
#include <cblas.h>

static void get_Yp(const value_type * __restrict__ Ap, const value_type * __restrict__ W,
		   value_type * __restrict__ Yp, const value_type * __restrict__ Dp,
		   const index_type height, const index_type width) {
  for (index_type i=0;i<width;i++) {
    Yp[height*width + i] = 2*DF(Ap[height*width + i])*(Dp[i] - F(Ap[height*width + i]));
  }

  for (index_type k=height-1;k+1>0;k--) {
    cblas_sgemv(CblasRowMajor, CblasTrans, width, width, 1,
		W+k*width*width, width, Yp+(k+1)*width, 1, 0, Yp+k*width, 1);
    for (index_type i=0;i<width;i++)
      Yp[k*width + i] *= DF(Ap[k*width + i]);
  }
}

#endif

int main() {
  constexpr index_type height=10, width=10000;

  value_type Ap[(height+1)*width];
  value_type W[height*width*width];
  value_type Yp[(height+1)*width];
  value_type Dp[width];

  get_Yp(Ap, W, Yp, Dp, height, width);
  printf("Done %f\n", Yp[3]);

  return 0;
}
