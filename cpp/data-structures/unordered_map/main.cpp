#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>

#define UNUSED_VAR(x) (void)x

template<typename A, typename B>
struct pair_hash {
  // This is from python's implementation of hashing a tuple
  size_t operator()(const std::pair<A, B> &thepair) const {
    std::hash<A> hasherA;
    std::hash<B> hasherB;
    size_t value = 0x345678;
    value = (1000003 * value) ^ hasherA(thepair.first);
    value = (1000003 * value) ^ hasherB(thepair.second);
    return value;
  }
};

int main(int argCount, char* argList[]) {
  UNUSED_VAR(argCount);
  UNUSED_VAR(argList);

  std::unordered_map<
    std::pair<std::string, std::string>
    , std::string
    , pair_hash<std::string, std::string>
    > my_map;

  my_map.insert({{"name", "d"}, "this is a test"});
  std::cout << my_map[{"name", "d"}] << std::endl;

  return 0;
}
