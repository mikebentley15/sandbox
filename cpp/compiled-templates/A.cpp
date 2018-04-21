#include "A.h"

template <typename T> A<T>::A() : m_val() {}
template <typename T> T A<T>::val() const { return m_val; }
template <typename T> void A<T>::val(const T& newval) { m_val = newval; }
template <typename T> void A<T>::val(T&& newval) { m_val = newval; }

// Declare publicly available types
template class A<int>;
template class A<float>;
