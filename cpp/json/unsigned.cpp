#include "nlohmann/json.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>

using nlohmann::json;

template <typename T>
constexpr std::string readable_type() {
#define TYPE_RETURN(type) \
  if constexpr (std::is_same_v<T, type>) { return #type; }

  TYPE_RETURN(char)
  TYPE_RETURN(short)
  TYPE_RETURN(int)
  TYPE_RETURN(long)
  TYPE_RETURN(long long)
  TYPE_RETURN(unsigned char)
  TYPE_RETURN(unsigned short)
  TYPE_RETURN(unsigned int)
  TYPE_RETURN(unsigned long)
  TYPE_RETURN(unsigned long long)

#undef TYPE_RETURN

  return "unknown";
}

template <typename T>
void print_interp(json &j) {
  std::cout << "\ninterpret data as " << readable_type<T>() << ":\n";
  for (const auto &[key, val] : j.items()) {
    T converted = val;
    std::cout
      << "  "
      << key
      << ": "
      << converted
      << std::endl;
  }
}

#define print_interp_macro(type, j) \
{ \
  std::cout << "\ninterpret data as " << readable_type<type>() << ":\n"; \
  for (const auto &[key, val] : j.items()) { \
    std::cout \
      << "  " \
      << key \
      << ": " \
      << val.get<type>() \
      << std::endl; \
  } \
}

int main() {
  std::string contents = R"(
{
  "negative-int": -3,
  "positive-int":  4,
  "zero-int":      0,
  "large-int":     8589934592
})";
  std::istringstream in(contents);

  json data;
  in >> data;

  // //print_interp<char              >(data);
  // print_interp<short             >(data);
  // print_interp<int               >(data);
  // print_interp<long              >(data);
  // print_interp<long long         >(data);
  // //print_interp<unsigned char     >(data);
  // print_interp<unsigned short    >(data);
  // print_interp<unsigned int      >(data);
  // print_interp<unsigned long     >(data);
  // print_interp<unsigned long long>(data);

  print_interp_macro(short             , data);
  print_interp_macro(int               , data);
  print_interp_macro(long              , data);
  print_interp_macro(long long         , data);
  print_interp_macro(unsigned short    , data);
  print_interp_macro(unsigned int      , data);
  print_interp_macro(unsigned long     , data);
  print_interp_macro(unsigned long long, data);

  std::cout << std::endl;

  return 0;
}
