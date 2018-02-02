#include <fstream>
#include <string>

bool file_exists(const std::string &filename) {
  std::ifstream in(filename);
  return in.good();
}

std::ofstream& open_new(std::ofstream &out, std::string prefix,
                        std::string suffix)
{
  std::string filename = prefix + suffix;
  unsigned int index = 0;
  while (file_exists(filename)) {
    index++;
    filename = prefix + "(" + std::to_string(index) + ")" + suffix;
  }
  out.rdbuf()->open(filename, std::ios_base::out);
  return out;
}

int main() {
  std::string prefix = "qbc";
  std::string suffix = ".txt";
  std::ofstream out;
  open_new(out, prefix, suffix);
  out << "hello world!\n";
  return 0;
}
