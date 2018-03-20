extern int missing_function(int a);

int defined_function(int a) {
  return missing_function(a);
}
