#include <cstdio>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <complex>

/* This will move to benchmark specific locations */
#define NUM_ARR 2
#define DIM 2

#define _LBX 0.0
#define _UBX 1.0
#define _LBY 0.0
#define _UBY 1.0

#define _xpos(_i, _dhx) (_LBX + _i*_dhx)
#define _ypos(_j, _dhy) (_LBY + _j*_dhy)


#include "array.h"

#include "misc.h"

#include "fullSimChar.h"

using namespace std ;

template <class T>
T fexactQuad(T x, T y, T t, T w) {
	return 1.0 + x*x + 3.0*y*y + 1.2*t ;
}

template <class T>
T fexactExpSin(T x, T y, T t, T w) {
	return exp((-M_PI*M_PI*t)/2) * sin (M_PI*((x+y)/2)) ;
}

template <class T>
T fexactExpCos(T x, T y, T t, T w) {
	return exp((-M_PI*M_PI*t)/2) * cos (M_PI*((x+y)/2)) ;
}

template <class T>
T fexactExpSinCos(T x, T y, T t, T w) {
	return exp((-M_PI*M_PI*t)/2) *( cos (M_PI*((x+y)/2)) +  sin (M_PI*((x+y)/2)) );
}


template <class T, class D>
void kernel_template( T* A, T* F, D& dh, D& dt, int& GT, int& nt, int& n,
						D (*fexactPtr)(D, D, D, D) ) {

	D alpha = 1.0 ;
	T(*A_tmp)[n+1][n+1] = (T(*)[n+1][n+1]) A ;
	T(*F_tmp)[n+1] = (T(*)[n+1]) F ;

	//printf("%lf %lf %d\n", dh, dt, n);
	//printf("%lf\n", (alpha*dt)/(dh*dh));

	for(int t=1; t<=nt; t++) {
		for(int i=0; i<=n; i++) {
			for(int j=0; j<=n; j++) {
				//printf("1: %d: %lf\n", t, A_tmp[t%2][i][j]);
				if(i>=1 && i<=n-1 && j>=1 && j<=n-1) {
					A_tmp[t%2][i][j] = (alpha*dt)/(dh*dh) * ( \
											A_tmp[(t+1)%2 ][i+1][j] + \
											A_tmp[(t+1)%2 ][i-1][j] + \
											A_tmp[(t+1)%2 ][i][j+1] + \
											A_tmp[(t+1)%2 ][i][j-1]  \
											) + \
											(1 - 4*(alpha*dt)/(dh*dh)) * A_tmp[(t+1)%2][i][j] + \
											F_tmp[i][j]*dt ;
				}
				//printf("2: %lf\n", A_tmp[1][0][0]);
				if(i==0) A_tmp[t%2][i][j] = fexactPtr(_xpos(i,dh), _ypos(j, dh), (GT+t)*dt, 0.0);
				if(i==n) A_tmp[t%2][i][j] =  fexactPtr(_xpos(i,dh), _ypos(j, dh), (GT+t)*dt, 0.0);
				if(j==0) A_tmp[t%2][i][j] = fexactPtr(_xpos(i,dh), _ypos(j, dh), (GT+t)*dt, 0.0);
				if(j==n) A_tmp[t%2][i][j] = fexactPtr(_xpos(i,dh), _ypos(j, dh), (GT+t)*dt, 0.0);

			}
		}
	}

	GT += nt ;
}


template <class T, class D>
void initialize( T* A, T* F, D& dh, int& n, D (*fexactPtr)(D, D, D, D)) {

	T(*A_tmp)[n+1][n+1] = (T(*)[n+1][n+1]) A;
	T(*F_tmp)[n+1] = (T(*)[n+1])F;
	
	for(int i=0; i<=n; i++) {
		for(int j=0; j<=n; j++) {
			for(int n=0; n<NUM_ARR; n++) {
				A_tmp[n][i][j] = fexactPtr(_xpos(i, dh), _ypos(j, dh), 0.0, 0.0);
			}
			F_tmp[i][j] = 1.2 - 2.0 - 6 ;
		}
	}
}

//void unit_test()
//{
//
//	int N=90 ;
//	double T=1.0 ;
//	double dh = 1.0/N;
//	double dt = 0.5*(dh*dh);
//	int GT = 0;
//	int nt = (int) (T/dt) ;
//	printf("T=%lf, dh=%lf, dt=%0.21f\n", T, dh, dh*dh*0.5);
//	printf("%d\n", nt);
//	ARRAY_DECL_3D(A, double, 2, N+1, N+1);
//	ARRAY_DECL_2D(F, double, N+1, N+1);
//	//for(int i=0; i<=N; i++) for(int j=0; j<=N; j++) (*F)[i][j] = 1.2-2.0-6;
//	//initialize<double, double>((double*)(*A)[0], dh, N, fexactQuad);
//	initialize<double, double>((double*)A, (double*)F, dh, N, fexactQuad);
//	kernel_template<double,double>((double*)A, (double*) F, dh, dt, GT, nt, N, fexactQuad);
//
//
//	ARRAY_FREE(A);
//	ARRAY_FREE(F);
//
//
//}

void setup_args(int argc, char** argv, _simArgs& args)
{
	parse_args(argc, argv, args);
	printf("%d, %lf, %lf\n", args.N, args.T, args.threshold);
}

int main(int argc, char** argv)
{
	//unit_test();
	_simArgs args ;
	setup_args(argc, argv, args);
	_fullSimChar<double, double>(args, fexactQuad);

	return 0;

}
