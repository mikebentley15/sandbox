/**
 * Author:       Michael Bentley
 * Date:         28 June 2021
 * Description:  Provides useful functionality from the standard C++ library
 *               here in Arduino space.
 *
 * Implemented
 * - from <type_traits>
 *   - remove_reference<T>
 *   - remove_reference_t<T>
 * - from <utility>
 *   - move()
 *   - pair<A, B> (not fully implemented - just simple)
 */
/** Defines move() similar to std::move() */
#ifndef mockstd_h
#define mockstd_h

/** remove_reference
 *
 * Copied from <type_traits> in GCC 10.2.0
 */
template<typename _Tp> struct remove_reference        { typedef _Tp   type; };
template<typename _Tp> struct remove_reference<_Tp&>  { typedef _Tp   type; };
template<typename _Tp> struct remove_reference<_Tp&&> { typedef _Tp   type; };
template<typename _Tp> using remove_reference_t = typename remove_reference<_Tp>::type;

/** cast to rvalue reference
 *
 * Copied from <bits/move.h> included from <utility> in GCC 10.2.0
 *
 * @brief  Convert a value to an rvalue.
 * @param  __t  A thing of arbitrary type.
 * @return The parameter cast to an rvalue-reference to allow moving it.
*/
template<typename _Tp>
  constexpr typename remove_reference<_Tp>::type&&
  move(_Tp&& __t) noexcept
{
  return static_cast<remove_reference_t<_Tp>&&>(__t);
}

// similar to std::pair<A,B> but simpler
template <typename A, typename B>
struct pair {
  A first;
  B second;

  // basic constructors
  pair() : first(), second() {}
  pair(const A& a, const B& b) : first(a), second(b) {}
  pair(A&& a, const B& b) : first(move(a)), second(b) {}
  pair(const A& a, B&& b) : first(a), second(move(b)) {}
  pair(A&& a, B&& b) : first(move(a)), second(move(b)) {}

  // enable copy and move
  pair(const pair &other) = default;
  pair(pair &&other) = default;
  pair& operator= (const pair &other) = default;
  pair& operator= (pair &&other) = default;
};



#endif // mockstd_h
