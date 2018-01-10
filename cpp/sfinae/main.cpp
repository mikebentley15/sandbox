#include <type_traits>
#include <iostream>
#include <string>

// elipsis version is at the bottom of the overload resolution priority.
// it will only be used if nothing else matches the overload.
void typeCompare_specialized(...)
{
  std::cout << "SHOULD NEVER BE CALLED!!!\n";
}

template <typename T>
typename std::enable_if<std::is_floating_point<T>::value, bool>::type
typeCompare_specialized(const T& lhs, const T& rhs)
{
  std::cout << "floating-point version\n";
  return (lhs - rhs) < 1e-10;
}

template <typename T>
typename std::enable_if<std::is_integral<T>::value, bool>::type
typeCompare_specialized(const T& lhs, const T& rhs)
{
  std::cout << "integral version\n";
  return lhs == rhs;
}

template <typename T>
auto typedCompare(const T& lhs, const T& rhs)
  -> typename std::enable_if<std::is_same<bool,decltype(typeCompare_specialized(lhs, rhs))>::value,bool>::type
{
  return typeCompare_specialized(lhs, rhs);
}

template <typename T>
auto typedCompare(const T& lhs, const T& rhs)
  -> typename std::enable_if<!std::is_same<bool,decltype(typeCompare_specialized(lhs, rhs))>::value,bool>::type
{
  std::cout << "catch-all version\n";
  return lhs == rhs;
}

int main()
{
  typedCompare(1, 1);
  typedCompare(1.0, 1.0);
  typedCompare(std::string("hello"), std::string("there"));

  return 0;
}
