
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <complex.h>
//#include <fftw3.h>
#include <unistd.h>


using namespace std;

typedef struct {
	int N ;
	double T ;
	double threshold;
} _simArgs ;


typedef struct {
	double dh ;
	double dt ;
	int fac ;
	int iter ;
} _discreteParams ;

enum _coarse { COARSE0 = 0, COARSE1 = 1, COARSE2 = 2} ; // will be used in powers of 2


/* function declarations */

template <class T>
T fexactQuad(T x, T y, T t, T w);

template <class T>
T fexactExpSin(T x, T y, T t, T w);

template <class T>
T fexactExpCos(T x, T y, T t, T w);

template <class T>
T fexactExpSinCos(T x, T y, T t, T w);

template <class T, class D>
void kernel_template( T* A, T* F, D& dh, D& dt, int& GT, int& nt, int& n,
						D (*fexactPtr)(D, D, D, D) ) ;

template <class T, class D>
void initialize(T* A, T* F, D& dh, int& n, D (*fexactPtr)(D, D, D, D)) ;

void setup( _discreteParams& dpar, _simArgs& args, _coarse fac) {
	dpar.dh = (_UBX - _LBX)/args.N ;
	dpar.dt = 0.25*dpar.dh*dpar.dh;
	dpar.iter = (int) (args.T/dpar.dt) ;

	dpar.fac = (int) pow(2, (int)fac);
	//cout << "******** Simulation Parammeters **************" << endl ;
	//cout << "Grid Size: " << args.N << " X " << args.N << endl ;
	//cout << "Space discretization : dh = " << dpar.dh << endl ;
	//cout << "Time discretization : dt = " << dpar.dt << endl ;
	//cout << "Iteration length : iter = " << dpar.iter << endl ;
	//cout << "Coarsening : f = " << dpar.fac << endl ;
	//cout << "**********************************************" << endl ;
}

void parse_args(int argc, char* argv[], _simArgs& args) {


	args.N = 10001+1;
	args.T = 1.0 ;
	args.threshold = 0.2 ;

	int v ;
	opterr = 0;

	while( (v = getopt(argc, argv, "n:t:d:h")) != -1) {
		switch(v) {
			case 'n':
				args.N = atoi(optarg);
				break ;
			case 't':
				args.T = strtod(optarg, NULL);
				break;
			case 'd':
				args.threshold = strtod(optarg, NULL);
				break;
			case 'h':
				printf("Arguments: [-n <ProbSize per dim> , -t <unit Tstep>, -d <threshold for trimming frequencies> | -h] \n");
				exit(0);
			case '?':
				fprintf(stderr, "Option -%c requires an argument. \n", optopt);
			default:
				abort();
		}
	}

}

