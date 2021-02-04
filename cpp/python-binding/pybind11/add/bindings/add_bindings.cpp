#include "add.h"

#include <pybind11/pybind11.h>

// the add_module definition
PYBIND11_MODULE(add_module, m) {
  m.doc() = "pybind11 example plugin"; // module docstring
  m.def("add", &add, "A function which adds two numbers");
}
