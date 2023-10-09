#include "tuple.hpp"

#include <iostream>
#include <tuple>

template<class... TupleTypes>
std::ostream& operator<<(std::ostream& out, const std::tuple<TupleTypes...>& t)
{
    out << "( ";
    std::apply(
        [&out](auto&&... args) {
            using swallow = int[]; // guarantees left to right order
            (void)swallow{0, (void(out << args << " "), 0)...};
        },
        t);
    return out << ")";
}

int main()
{
    auto a = std::make_tuple(1, 5UL, std::string("hello"), true, 3.14159);
    auto hashvalue = std::hash<decltype(a)>()(a);
    std::cout << "Tuple " << a << " has hash " << hashvalue << "\n";
    return 0;
}
