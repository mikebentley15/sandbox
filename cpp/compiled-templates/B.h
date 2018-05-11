#ifndef B_H
#define B_H

// This class has all functions inlined, but a few types are pre-instantiated
// in B.cpp, so that we don't have to compile this class so many times.
//
// At least that was the hope.  It turns out that forward declaring a
// specialization also means that the class is not guaranteed to follow this
// structure.  That means the specialization can have things such as more
// member variables, making the instantiated objects be of different size.
//
// So, this approach does NOT work, but the approach in A does work.  The
// downside to the approach with A is that you cannot instantiate arbitrary
// types.  I suppose maybe you could if you also included the cpp file.  There
// may be a way to create files like "A.h" and "A.hpp" where A.h would just
// have the template class definition, but not the implemetation.  The
// implementation would reside in A.hpp.  But if you want to use arbitrary
// types, you would include A.hpp.  This would be bad if you include A.hpp and
// then used one of the predefined types since now you are using a local one
// instead of the one in the shared library, or you could maybe get duplicate
// symbols.
//
// Well, at least you don't get duplicate symbols...  This was tested using
// A2.cpp which instantiates the same class types.
template <typename T>
class B {
public:
  B() : m_val() {}
  T val() const { return m_val; }
  void val(const T& newval) { m_val = newval; }
  void val(T&& newval) { m_val = newval; }
private:
  T m_val;
};

// forward declare that these specializations exist
#ifndef B_CPP
template <> class B<int>;
template <> class B<float>;
#endif

#endif  // B_H
