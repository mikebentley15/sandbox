#ifndef RETURNS_VOID_H
#define RETURNS_VOID_H

#include <type_traits>

template <typename Func>
struct return_type { using type = decltype(std::declval<Func>()()); };

template <typename Func>
using return_type_t = typename return_type<Func>::type;

template <typename Func>
struct returns_void : std::is_void<return_type_t<Func>> { };

template <typename Func>
inline constexpr bool returns_value_v = ! returns_void<Func>::value;

template <typename Func>
inline constexpr bool returns_void_v = returns_void<Func>::value;


#endif // RETURNS_VOID_H
