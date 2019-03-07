
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <complex.h>
//#include <fftw3.h>
#include <unistd.h>



using namespace std ;

template <class T, class D>
void _fullSimChar(_simArgs& args, D (*fexactPtr)(D, D, D, D))
{
	int GT=0; // Tracks the global time of the simulation
	_discreteParams dpar ;
	setup(dpar, args, COARSE0);

	/* Declare and initialize */
	ARRAY_DECL_3D(A, T, NUM_ARR, args.N+1, args.N+1)
	ARRAY_DECL_2D(F, T, args.N+1, args.N+1)
	initialize<T, D>((T*)A, (D*)F, dpar.dh, args.N, fexactPtr);

	kernel_template<T,D>((T*) A, (T*) F, dpar.dh, dpar.dt, GT, dpar.iter, args.N, fexactPtr);




	ARRAY_FREE(A);
}

