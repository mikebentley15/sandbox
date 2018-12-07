#include <string>

class B {} b_instance_1, b_instance_2;

template<typename T, int I = 3>
class
A : public B
{
  public:
    A() : B() {}
  private:
    std::string in() const;
  protected:
    std::string _out;
  private:
    std::string _in;
};
