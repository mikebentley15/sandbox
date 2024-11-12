#ifndef GENERATOR_MACROS_H
#define GENERATOR_MACROS_H

class _base_generator {
protected:
  int _line_no;
public:
  _base_generator() = default;
};

#define GENERATOR(name) class name : public _base_generator


#endif // GENERATOR_MACROS_H
