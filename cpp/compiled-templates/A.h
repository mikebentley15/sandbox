#ifndef A_H
#define A_H

template <typename T>
class A {
public:
  A();
  T val() const { return m_val; }
  void val(const T& newval);
  void val(T&& newval);
private:
  T m_val;
};

#endif // A_H
