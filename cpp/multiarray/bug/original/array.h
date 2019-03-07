#ifndef ARRAY_H_
#define ARRAY_H_

#ifndef PADDING
#define PADDING 0
#endif

// #ifndef DATA_TYPE
// #define double
// #endif

//# define _PB_N POLYBENCH_LOOP_BOUND(N,n)

# ifndef DATA_TYPE
#  define DATA_TYPE double
#  define DATA_PRINTF_MODIFIER "%0.2lf "
# endif

typedef struct {
	int min ;
	int max ;
} exp_t ;


static
void *
xmalloc (size_t num)
{
  void* ne = NULL;
  int ret = posix_memalign (&ne, 32, num);
  if (! ne || ret)
    {
      fprintf (stderr, "[PolyBench] posix_memalign: cannot allocate memory");
      exit (1);
    }
  return ne;
}

void* polybench_alloc_data(unsigned long long int n, int elt_size);

void* polybench_alloc_data(unsigned long long int n, int elt_size)
{
  /// FIXME: detect overflow!
  size_t val = n;
  val *= elt_size;
  void* ret = xmalloc (val);

  return ret;
}
// {
//   /// FIXME: detect overflow!
//   size_t val = n;
//   val *= elt_size;
//   void* ret = xmalloc (val);

//   return ret;
// }

/* Macros for using arrays in the function prototypes. */
# define ARRAY_1D(var, dim1) (*var)[dim1 + PADDING]
# define ARRAY_2D(var, dim1, dim2) (*var)[dim1 + PADDING][dim2 + PADDING]
# define ARRAY_3D(var, dim1, dim2, dim3) (*var)[dim1 + PADDING][dim2 + PADDING][dim3 + PADDING]
# define ARRAY_4D(var, dim1, dim2, dim3, dim4) (*var)[dim1 + PADDING][dim2 + PADDING][dim3 + PADDING][dim4 + PADDING]
# define ARRAY_5D(var, dim1, dim2, dim3, dim4, dim5) (*var)[dim1 + PADDING][dim2 + PADDING][dim3 + PADDING][dim4 + PADDING][dim5 + PADDING]
# define ARRAY_6D(var, dim1, dim2, dim3, dim4, dim5, dim6) (*var)[dim1 + PADDING][dim2 + PADDING][dim3 + PADDING][dim4 + PADDING][dim5 + PADDING][dim6 + PADDING]
# define ARRAY_7D(var, dim1, dim2, dim3, dim4, dim5, dim6, dim7) (*var)[dim1 + PADDING][dim2 + PADDING][dim3 + PADDING][dim4 + PADDING][dim5 + PADDING][dim6 + PADDING][dim7 + PADDING]
# define ARRAY_8D(var, dim1, dim2, dim3, dim4, dim5, dim6, dim7, dim8) (*var)[dim1 + PADDING][dim2 + PADDING][dim3 + PADDING][dim4 + PADDING][dim5 + PADDING][dim6 + PADDING][dim7 + PADDING][dim8 + PADDING]


# define ARRAY_ALLOC_1D(n1, type)	\
  (type(*)[n1 + PADDING])polybench_alloc_data (n1 + PADDING, sizeof(type))
# define ARRAY_ALLOC_2D(n1, n2, type)		\
  (type(*)[n1 + PADDING][n2 + PADDING])polybench_alloc_data ((n1 + PADDING) * (n2 + PADDING), sizeof(type))
# define ARRAY_ALLOC_3D(n1, n2, n3, type)		\
  (type(*)[n1 + PADDING][n2 + PADDING][n3 + PADDING])polybench_alloc_data ((n1 + PADDING) * (n2 + PADDING) * (n3 + PADDING), sizeof(type))
# define ARRAY_ALLOC_4D(n1, n2, n3, n4, type)			\
  (type(*)[n1 + PADDING][n2 + PADDING][n3 + PADDING][n4 + PADDING])polybench_alloc_data ((n1 + PADDING) * (n2 + PADDING) * (n3 + PADDING) * (n4 + PADDING), sizeof(type))
# define ARRAY_ALLOC_5D(n1, n2, n3, n4, n5, type)		\
  (type(*)[n1 + PADDING][n2 + PADDING][n3 + PADDING][n4 + PADDING][n5 + PADDING])polybench_alloc_data ((n1 + PADDING) * (n2 + PADDING) * (n3 + PADDING) * (n4 + PADDING) * (n5 + PADDING), sizeof(type))
# define ARRAY_ALLOC_6D(n1, n2, n3, n4, n5, n6, type)                    \
  (type(*)[n1 + PADDING][n2 + PADDING][n3 + PADDING][n4 + PADDING][n5 + PADDING][n6 + PADDING])polybench_alloc_data ((n1 + PADDING) * (n2 + PADDING) * (n3 + PADDING) * (n4 + PADDING) * (n5 + PADDING) * (n6 + PADDING), sizeof(type))

# define ARRAY_ALLOC_7D(n1, n2, n3, n4, n5, n6, n7, type)                    \
  (type(*)[n1 + PADDING][n2 + PADDING][n3 + PADDING][n4 + PADDING][n5 + PADDING][n6 + PADDING][n7 + PADDING])polybench_alloc_data ((n1 + PADDING) * (n2 + PADDING) * (n3 + PADDING) * (n4 + PADDING) * (n5 + PADDING) * (n6 + PADDING) * (n7 + PADDING), sizeof(type))

# define ARRAY_ALLOC_8D(n1, n2, n3, n4, n5, n6, n7, n8, type)                    \
  (type(*)[n1 + PADDING][n2 + PADDING][n3 + PADDING][n4 + PADDING][n5 + PADDING][n6 + PADDING][n7 + PADDING][n8 + PADDING])polybench_alloc_data ((n1 + PADDING) * (n2 + PADDING) * (n3 + PADDING) * (n4 + PADDING) * (n5 + PADDING) * (n6 + PADDING) * (n7 + PADDING) * (n8 + PADDING), sizeof(type))

#  define ARRAY_DECL_1D(var, type, dim1)		\
  type ARRAY_1D(var, dim1); \
  var = ARRAY_ALLOC_1D(dim1, type);
#  define ARRAY_DECL_2D(var, type, dim1,dim2)       \
  type ARRAY_2D(var, dim1, dim2);                   \
  var = ARRAY_ALLOC_2D(dim1, dim2, type);
#  define ARRAY_DECL_3D(var, type, dim1,dim2,dim3)  \
  type ARRAY_3D(var, dim1, dim2, dim3);             \
  var = ARRAY_ALLOC_3D(dim1, dim2, dim3, type);
#  define ARRAY_DECL_4D(var, type, dim1, dim2, dim3, dim4) \
  type ARRAY_4D(var, dim1, dim2, dim3, dim4);              \
  var = ARRAY_ALLOC_4D(dim1, dim2, dim3, dim4, type);
#  define ARRAY_DECL_5D(var, type, dim1, dim2, dim3, dim4, dim5) \
  type ARRAY_5D(var, dim1, dim2, dim3, dim4, dim5);              \
  var = ARRAY_ALLOC_5D(dim1, dim2, dim3, dim4, dim5, type);
#  define ARRAY_DECL_6D(var, type, dim1, dim2, dim3, dim4, dim5, dim6) \
  type ARRAY_6D(var, dim1, dim2, dim3, dim4, dim5, dim6);                   \
  var = ARRAY_ALLOC_6D(dim1, dim2, dim3, dim4, dim5, dim6, type);
#  define ARRAY_DECL_7D(var, type, dim1, dim2, dim3, dim4, dim5, dim6, dim7) \
  type ARRAY_7D(var, dim1, dim2, dim3, dim4, dim5, dim6, dim7);                   \
  var = ARRAY_ALLOC_7D(dim1, dim2, dim3, dim4, dim5, dim6, dim7, type);
#  define ARRAY_DECL_8D(var, type, dim1, dim2, dim3, dim4, dim5, dim6, dim7, dim8) \
  type ARRAY_8D(var, dim1, dim2, dim3, dim4, dim5, dim6, dim7, dim8);                   \
  var = ARRAY_ALLOC_8D(dim1, dim2, dim3, dim4, dim5, dim6, dim7, dim8, type);

#define ARRAY_FREE(_A) free(_A)

#if 0
/* Array initialization. */
static
void init_array_1d(int n, int num_arr, DATA_TYPE C[num_arr][n])
{
  int i, j;

  for(i=0; i<num_arr; i++)
	  for (j = 0; j < n; j++)
	  	 C[i][j] = ((double)(j+1))/n ;
    	 //C[i][j] = 42;
		 //C[i][j] =  rand()%n; //((double)(j+1))/n ;
}

#if 0
static
void init_array_2d(int n, int num_arr, DATA_TYPE C[num_arr][n][n])
{
  int i, j, k;

  for (i=0; i<num_arr; i++)
	  for (j = 0; j < n; j++)
    	for (k = 0; k < n; k++)
  			C[i][j][k] = 42 ;
   //   C[i][j] = (i+0.54)*(j+0.43)/n;
}
#endif

static
void init_array_2d(int num_arr, int dim, int n[num_arr][dim], int nmax0, int nmax1, 
				   DATA_TYPE C[num_arr][nmax0][nmax1])
{

	for(int num=0; num<num_arr; num++) {
		for(int i=0; i<nmax0; i++) {
			for(int j=0; j<nmax1; j++) {
				if(i < n[num][0] && j < n[num][1])
					//C[num][i][j] = 1 + pow(((i+0.54)/nmax0),2) + pow(((j+0.43)/nmax1),2) ;
					C[num][i][j] = 2+(i+0.54)*(j+0.43)/(nmax0*nmax1) ;
					//C[num][i][j] = 42 ;
				else
					C[num][i][j] = 0.0 ;
			}
		}
	}


}


static
void init_array_3d(int num_arr, int dim, int n[num_arr][dim], int nmax0, int nmax1, int nmax2,
				   DATA_TYPE C[num_arr][nmax0][nmax1][nmax2])
{

	for(int num=0; num<num_arr; num++) {
		for(int i=0; i<nmax0; i++) {
			for(int j=0; j<nmax1; j++) {
				for(int k=0; k<nmax2; k++) {
					if(i < n[num][0] && j < n[num][1])
						//C[num][i][j] = 1 + pow(((i+0.54)/nmax0),2) + pow(((j+0.43)/nmax1),2) ;
						C[num][i][j][k] = 2+(i+0.54)*(j+0.43)*(k+54)/(nmax0*nmax1*nmax2) ;
						//C[num][i][j] = 42 ;
					else
						C[num][i][j][k] = 0.0 ;
				}
			}
		}
	}


}

#if 0
static
void init_array_3d(int n, int num_arr, DATA_TYPE C[num_arr][n][n][n])
{
  int i, j, k, l;

  for (i = 0; i < num_arr; i++)
    for (j = 0; j < n; j++)
      for (k = 0; k < n; k++)
	  	for (l = 0; l < n; l++)
	        C[i][j][k][l] = 42;
}
#endif


/* DCE code. Must scan the entire live-out data.
   Can be used also to check the correctness of the output. */
static
void print_array_1d(int n, DATA_TYPE C[n])
{
  int i;

  for (i = 0; i < n; i++) {
	fprintf (stderr, DATA_PRINTF_MODIFIER, C[i]);
	if (i % 20 == 0) fprintf (stderr, "\n");
  }
  fprintf (stderr, "\n");
}

static
void print_array_2d(int n, DATA_TYPE C[n][n])
{
  int i, j;

  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++) {
	fprintf (stderr, DATA_PRINTF_MODIFIER, C[i][j]);
	if (i % 20 == 0) fprintf (stderr, "\n");
    }
  fprintf (stderr, "\n");
}

static
void print_array_3d(int n, DATA_TYPE C[n][n][n])
{
  int i, j, k;

  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++)
    for (k = 0; k < n; k++) {
	fprintf (stderr, DATA_PRINTF_MODIFIER, C[i][j][k]);
	if (j % 20 == 0) fprintf (stderr, "\n");
    }
  fprintf (stderr, "\n");
}

static
int compare_array(int sz,
                  double A[],
                  double B[],
                  double dp) {
  int ret = 0;
  for(int i=0; i<sz; i++) {
    //ret |= COMPARE(A[i], B[i], dp);
	//printf("%d: i=%d\n",sz, i);
	if(i==404) printf("%0.21f %0.21f\n", A[i], B[i]);
    assert(COMPARE(A[i], B[i], dp)==0);
  }
  return ret;
}

static
int compare_array_1d(int n1,
                     double A[n1],
                     double B[n1],
                     double dp) {
  int ret = 0;
  double *Aptr = (double*)(A);
  double *Bptr = (double*)(B);
  int sz = n1;
  for(int i=0; i<sz; i++) {
    ret |= COMPARE(Aptr[i], Bptr[i], dp);
  }
  return ret;
}

static
int compare_array_2d(int n1, int n2,
                     double A[n1][n2],
                     double B[n1][n2],
                     double dp) {
  int ret = 0;
  double *Aptr = (double*)(A);
  double *Bptr = (double*)(B);
  int sz = n1 * n2;
  for(int i=0; i<sz; i++) {
    ret |= COMPARE(Aptr[i], Bptr[i], dp);
  }
  return ret;
}

static
int compare_array_3d(int n1, int n2, int n3,
                     double A[n1][n2][n3],
                     double B[n1][n2][n3],
                     double dp) {
  int ret = 0;
  double *Aptr = (double*)(A);
  double *Bptr = (double*)(B);
  int sz = n1 * n2 * n3;
  for(int i=0; i<sz; i++) {
    ret |= COMPARE(Aptr[i], Bptr[i], dp);
  }
  return ret;
}

static
void memset_array_1d(int n1,
                     double A[n1],
                     int ilo, int ihi,
                     double val) {
	int i;
	for(i=ilo; i<=ihi; i++) {
    A[i] = val;
	}
}

static
void memset_array_2d(int n1, int n2,
                     double A[n1][n2],
                     int ilo, int ihi,
                     int jlo, int jhi,
                     double val) {
	int i, j ;
	for(i=ilo; i<=ihi; i++) {
		for(j=jlo; j<=jhi; j++) {
			A[i][j] = val ;
		}
	}
}

static
void memset_array_3d(int n1, int n2, int n3,
                     double A[n1][n2][n3],
                     int ilo, int ihi,
                     int jlo, int jhi,
                     int klo, int khi,
                     double val) {
	int i, j, k;
	for(i=ilo; i<=ihi; i++) {
		for(j=jlo; j<=jhi; j++) {
      for(k=klo; k<=khi; k++) {
        A[i][j][k] = val;
      }
    }
	}
}

static
void copy_patch_array_1d(int an1,
                         double In[an1],
                         int alo1, int ahi1,
                         int bn1,
                         double Out[bn1],
                         int blo1, int bhi1) {
  assert(ahi1 - alo1 == bhi1 - blo1);
  for(int i1 = 0; i1<ahi1-alo1; i1++) {
    Out[blo1 + i1] = In[alo1 + i1];
  }
}

static
void copy_patch_array_2d(int an1, int an2,
                         double In[an1][an2],
                         int alo1, int ahi1,
                         int alo2, int ahi2,
                         int bn1, int bn2,
                         double Out[bn1][bn2],
                         int blo1, int bhi1,
                         int blo2, int bhi2) {
  assert(ahi1 - alo1 == bhi1 - blo1);
  assert(ahi2 - alo2 == bhi2 - blo2);
  for(int i1 = 0; i1<ahi1-alo1; i1++) {
    for(int i2 = 0; i2<ahi2-alo2; i2++) {
      Out[blo1 + i1][blo2+i2] = In[alo1 + i1][alo2+i2];
    }
  }
}

static
void copy_patch_array_3d(int an1, int an2, int an3,
                         double In[an1][an2][an3],
                         int alo1, int ahi1,
                         int alo2, int ahi2,
                         int alo3, int ahi3,
                         int bn1, int bn2, int bn3,
                         double Out[bn1][bn2][bn3],
                         int blo1, int bhi1,
                         int blo2, int bhi2,
                         int blo3, int bhi3) {
  assert(ahi1 - alo1 == bhi1 - blo1);
  assert(ahi2 - alo2 == bhi2 - blo2);
  assert(ahi3 - alo3 == bhi3 - blo3);
  for(int i1 = 0; i1<ahi1-alo1; i1++) {
    for(int i2 = 0; i2<ahi2-alo2; i2++) {
      for(int i3 = 0; i3<ahi3-alo3; i3++) {
        Out[blo1 + i1][blo2+i2][blo3+i3] = In[alo1 + i1][alo2+i2][alo3+i3];
      }
    }
  }
}

#if 0
static
int exponent_range_array_1d(int n1,
                            double A[n1]) {
  int ret = 1;
  //@todo @bug implement
  int maxValue = get_exponent(A[1]);
  int minValue = get_exponent(A[1]);
  for(int i=1; i<n1-1; i++) {
  	if(get_exponent(A[i]) > maxValue) maxValue = get_exponent(A[i]);
	if(get_exponent(A[i]) < minValue) minValue = get_exponent(A[i]);
  }
  int diff = maxValue - minValue ;
  ret = diff+1 ;
  return ret;
}
#endif

static
exp_t exponent_range_array_1d(int ndim0, int nmax0, double A[nmax0])
{

	int maxValue = get_exponent(A[0]);
	int minValue = get_exponent(A[0]);
	int expVal ;
	exp_t ret ;
	ret.min = minValue ; ret.max = maxValue ;

	for(int i=1; i<ndim0-1; i++) {
		expVal = get_exponent(A[i]);
		if(expVal > maxValue) maxValue = expVal ;
		if(expVal < minValue) minValue = expVal ;
	}

	ret.min = minValue ;
	ret.max = maxValue ;
	return ret ;
		
}

static
exp_t exponent_range_array_2d(int ndim0, int ndim1, int nmax0, int nmax1, 
							double A[nmax0][nmax1])
{
	//int ret = 1;
	int maxValue = get_exponent(A[0][0]);
	int minValue = get_exponent(A[0][0]);
	int expVal ;
	exp_t ret ;
	ret.min = minValue ; ret.max = maxValue ;

	for(int i=0; i<ndim0-1; i++) {
		for(int j=0; j<ndim1-1; j++) {
			expVal = get_exponent(A[i][j]);
			if(expVal > maxValue) maxValue = expVal ;
			if(expVal < minValue) minValue = expVal ;
		}
	}
	ret.min = minValue ;
	ret.max = maxValue ;
	//int diff = maxValue - minValue ;
	//ret = diff + 1 ;

	return ret ;
}

static
exp_t exponent_range_array_3d(int ndim0, int ndim1, int ndim2, int nmax0, int nmax1, int nmax2,
							double A[nmax0][nmax1][nmax2])
{

	int maxValue = get_exponent(A[0][0][0]);
	int minValue = get_exponent(A[0][0][0]);
	int expVal ;
	exp_t ret ;
	ret.min = minValue ; ret.max = maxValue ;

	for(int i=1; i<ndim0-1; i++) {
		for(int j=1; j<ndim1-1; j++) {
			for(int k=1; k<ndim2-1; k++) {
				expVal = get_exponent(A[i][j][k]);
				if(expVal > maxValue) maxValue = expVal ;
				if(expVal < minValue) minValue = expVal ;
			}
		}
	}

	ret.min = minValue ;
	ret.max = maxValue ;

	return ret ;

}

#if 0
static
int exponent_range_array_2d(int n1, int n2,
                            double A[n1][n2]) {
  int ret = 1;
  //@todo @bug implement
  int maxValue = get_exponent(A[1][1]);
  int minValue = get_exponent(A[1][1]);
  for(int i=1; i<n1-1; i++) {
  	for(int j=1; j<n2-1; j++) {
		if(get_exponent(A[i][j]) > maxValue) maxValue = get_exponent(A[i][j]);
		if(get_exponent(A[i][j]) < minValue) minValue = get_exponent(A[i][j]);
	}
  }
  int diff = maxValue - minValue ;
  ret = diff + 1;
  return ret;
}
#endif

#if 0
static
int exponent_range_array_3d(int n1, int n2, int n3,
                            double A[n1][n2][n3]) {
  int ret = 1;
  //@todo @bug implement
  int maxValue = get_exponent(A[1][1][1]);
  int minValue = get_exponent(A[1][1][1]);
  for(int i=1; i<n1-1; i++) {
  	for(int j=1; j<n2-1; j++) {
		for(int k=1; k<n3-1; k++) {
			if(get_exponent(A[i][j][k]) > maxValue) maxValue = get_exponent(A[i][j][k]) ;
			if(get_exponent(A[i][j][k]) < minValue) minValue = get_exponent(A[i][j][k]) ;
		}
	}
  }
  int diff = maxValue - minValue ;
  ret = diff + 1;
  return ret;
}

#endif
#endif

#endif // ARRAY_H_

