#include <algorithm>
#include <map>
#include <iostream>
#include <string_view>

template<size_t N>
struct StringLiteral {
    constexpr StringLiteral(const char (&str)[N]) {
        std::copy_n(str, N, value);
    }
    
    char value[N];

    constexpr bool operator== (const StringLiteral &other) {
        return std::equal(std::begin(value), std::end(value),
                          std::begin(other.value), std::end(other.value));
    }
};

struct MyTuple {
    static constexpr std::string_view _name = "MyTuple";
    static constexpr char _name_1[] = "x";
    static constexpr char _name_2[] = "y";
    static constexpr char _name_3[] = "name";

    static constexpr std::string_view _names[] = { "x", "y", "name", };

    int x;
    int y;
    std::string name;

    template <size_t> auto& get();
    template <StringLiteral> auto& get();
    template <size_t> auto get() const;
    template <StringLiteral> auto get() const;
};

template <> inline auto& MyTuple::get<0>() { return x; }
template <> inline auto& MyTuple::get<1>() { return y; }
template <> inline auto& MyTuple::get<2>() { return name; }
template <> inline auto& MyTuple::get<"x">() { return x; }
template <> inline auto& MyTuple::get<"y">() { return y; }
template <> inline auto& MyTuple::get<"name">() { return name; }

template <> inline auto MyTuple::get<0>() const { return x; }
template <> inline auto MyTuple::get<1>() const { return y; }
template <> inline auto MyTuple::get<2>() const { return name; }
template <> inline auto MyTuple::get<"x">() const { return x; }
template <> inline auto MyTuple::get<"y">() const { return y; }
template <> inline auto MyTuple::get<"name">() const { return name; }

std::ostream& operator<<(std::ostream &out, const MyTuple &t) {
    return out
        << t._name << "{"
        << MyTuple::_names[0] << ": " << t.get<0>() << ", "
        << MyTuple::_names[1] << ": " << t.get<1>() << ", "
        << MyTuple::_names[2] << ": " << t.get<2>() << ", "
        << "}";
    return out;
}

int main() {
    MyTuple x {.name = "my instance"};
    std::cout << x._names[2] << "\n";
    std::cout << x.get<"x">() << "\n";
    std::cout << x << "\n";
    return 0;
}
