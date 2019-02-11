#include <iostream>

template <typename T>
class Array2D {
public:
  Array2D(T* arr, int n, int m);             // constructor
  Array2D(Array2D<T> &&other);               // move constructor
  Array2D<T>& operator=(Array2D<T> &&other); // move assignment operator

  T& operator() (int i, int j);
  T operator() (int i, int j) const;

  int n() const { return _n; }
  int m() const { return _m; }
  T* arr() const { return _arr; }

private:
  int _n;
  int _m;
  T* _arr;
};

template <typename T>
Array2D<T>::Array2D(T* arr, int n, int m) : _n(n), _m(m), _arr(arr) {}

template <typename T>
Array2D<T>::Array2D(Array2D<T> &&other) {
  *this = std::move(other);
}

template <typename T>
Array2D<T>& Array2D<T>::operator=(Array2D<T> &&other) {
  _arr = other._arr;
  _n = other._n;
  _m = other._m;
  other._arr = nullptr;
  other._n = -1;
  other._m = -1;
}

template <typename T>
T Array2D<T>::operator() (int i, int j) const {
  // TODO: check for out of bounds?
  return ((T(*)[_m])_arr)[i][j];
}

template <typename T>
T& Array2D<T>::operator() (int i, int j) {
  // TODO: check for out of bounds?
  return ((T(*)[_m])_arr)[i][j];
}

void printArray2Ddouble(const Array2D<double> &a) {
  for (int i = 0; i < a.n(); i++) {
    for (int j = 0; j < a.m(); j++) {
      std::cout << "(" << i << ", " << j << "): "
                << a(i, j) << std::endl;
    }
  }
}

template<typename T>
void printArray2D(const Array2D<T> &a) {
  for (int i = 0; i < a.n(); i++) {
    for (int j = 0; j < a.m(); j++) {
      std::cout << "(" << i << ", " << j << "): "
                << a(i, j) << std::endl;
    }
  }
}

template<typename T>
void setArray2D(Array2D<T> &a) {
  for (int i = 0; i < a.n(); i++) {
    for (int j = 0; j < a.m(); j++) {
      a(i, j) = i * a.m() + j;
    }
  }
}

int main(int argCount, char* argList[]) {
  int n = 5;
  int m = 2;
  if (argCount > 1) {
    n = std::atoi(argList[1]);
  }
  if (argCount > 2) {
    m = std::atoi(argList[2]);
  }
  double *d = new double[n*m];
  Array2D<double> a(d, n, m);
  setArray2D(a);
  //printArray2Ddouble(a);
  printArray2D(a);
  delete d;
  return 0;
}
