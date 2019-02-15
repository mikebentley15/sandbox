float dot(float* x, float* y, int n) {
  float sum = 0.0;
  for (int i = 0; i < n; i++) {
    sum += x[i] * y[i];
  }
  return sum;
}
