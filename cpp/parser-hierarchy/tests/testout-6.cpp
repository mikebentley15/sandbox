template < typename _UIntType, size_t __w, bool = __w < static_cast < size_t > (std :: numeric_limits < _UIntType > :: digits ) > struct _Shift {
  static const _UIntType __value = 0;
};
