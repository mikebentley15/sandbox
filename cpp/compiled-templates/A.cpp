#include "A.h"

template <typename T> A<T>::A() : m_val() {}
template <typename T> void A<T>::val(const T& newval) { m_val = newval; }
template <typename T> void A<T>::val(T&& newval) { m_val = newval; }

// Declare publicly available types
// These instantiate these types and includes them into the object file as weak
// symbols.  That makes sense why multiple don't cause problems and only one
// will ever be used.  This approach allows the optimization of only compiling
// template classes once (except for functions you want to be inlined).  For
// classes that are used in hundreds or thousands of object files, especially
// if the template class  is large, then this could be quite a good
// compilation optimization.
template class A<int>;
template class A<float>;
