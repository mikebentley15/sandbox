#include <iostream>

#include <climits>

int main(void) {
  std::cout << "CHAR_MIN:  " << CHAR_MIN << std::endl;
  std::cout << "-CHAR_MIN: " << -CHAR_MIN << std::endl;
  std::cout << "(char)(-CHAR_MIN): " << (char) (-CHAR_MIN) << std::endl;
  std::cout << "(int)((char)(-CHAR_MIN)): " << (int)((char) (-CHAR_MIN)) << std::endl;
  std::cout << "CHAR_MAX:  " << CHAR_MAX << std::endl;
  std::cout << "(char)(-CHAR_MIN) == CHAR_MIN:  " << 
    ((char)(-CHAR_MIN) == CHAR_MIN ? "true" : "false") << std::endl;;
  return 0;
}
