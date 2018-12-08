// This example fails because it has a "<" less than operator within a template
// declaration
//
// How am I expected to tell the difference between a template parameter
// beginning and a less than operator?
//
// Likewise, how do I tell the difference between a template ending and a
// greater than operator?
//
// I don't want to support wierd grammar like this, but what can I do if this
// is from the file I want to parse?

template<typename _UIntType, size_t __w,
  bool = __w < static_cast<size_t>
 (std::numeric_limits<_UIntType>::digits)>
  struct _Shift
  { static const _UIntType __value = 0; };

