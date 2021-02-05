template <typename A, typename B>
struct pair {
  A first;
  B second;
};

void g(int k) {
  pair<bool, bool> arr[k] {};
}
