#include "A.h"

#include <iostream>
#include <boost/filesystem.hpp>

void A::print() const {
  std::cout << "A::print()" << std::endl;
  std::cout << "  Current directory listing:\n";
  namespace fs = boost::filesystem;
  auto curdir = fs::system_complete(fs::path("."));
  fs::directory_iterator end_iter;
  for (fs::directory_iterator dir_iter(curdir);
       dir_iter != end_iter;
       ++dir_iter)
  {
    std::cout << "    " << dir_iter->path().filename() << "\n";
  }
  std::cout << std::endl;
}
