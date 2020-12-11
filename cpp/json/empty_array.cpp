#include "nlohmann/json.hpp"

#include <iostream>

using nlohmann::json;

int main() {
  json empty;
  auto empty_array = json::array();
  std::cout << "empty:       " << empty << "\n"
               "empty_array: " << empty_array << "\n";
  return 0;
}
