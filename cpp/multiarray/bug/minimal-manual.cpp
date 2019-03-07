const int NUM_ARR = 2;

template <class T>
void initialize(T* A, const int n) {
  T(*A_tmp)[n+1][n+1] = (T(*)[n+1][n+1]) A;
  for(int i=0; i<=n; i++) {
    for(int j=0; j<=n; j++) {
      for(int k=0; k<NUM_ARR; k++) {
        A_tmp[k][i][j] = 0.0;
      }
    }
  }
}

int main(int argc, char** argv) {
  const int N = 100;
  double A[(N+1)*(N+1)*NUM_ARR];
  initialize<double>((double*)A, N);
  return 0;
}
