template <typename T> struct A { };

int k = 3;

void g() {
  A<float> arr[k] {};
}
