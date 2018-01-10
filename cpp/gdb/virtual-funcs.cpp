#include <stdio.h>

class BaseTest {
  public:
  int a;

  BaseTest(): a(2){}

  virtual int gB() {
    return a;
  };
};

class SubTest: public BaseTest {
  public:
  int b;

  SubTest(): b(4){}
};

class TriTest: public BaseTest {
  public:
  int c;
  TriTest(): c(42){}
};

class EvilTest: public TriTest {
  public:
  virtual int gB(){
    return c;
  }
};

int main(){
  EvilTest * t2 = new EvilTest;

  TriTest * t3 = t2;

  printf("%d\n",t3->gB());
  printf("%d\n",t2->gB());
  return 0;
}
