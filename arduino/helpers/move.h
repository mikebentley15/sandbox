/** Defines move() similar to std::move() */
#ifndef move_h
#define move_h

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

#endif // move_h
