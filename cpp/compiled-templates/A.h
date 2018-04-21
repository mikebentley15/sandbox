template <typename T>
class A {
public:
  A();
  T val() const;
  void val(const T& newval);
  void val(T&& newval);
private:
  T m_val;
};
