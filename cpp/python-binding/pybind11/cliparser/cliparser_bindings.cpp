#include <CliParser.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

PYBIND11_MODULE(cliparser, m) {
  m.doc() = "alternative command-line parser";
  py::class_<CliParser>(m, "CliParser")
    .def(py::init<>())
    .def("args", &CliParser::args)
    .def("program_name", &CliParser::program_name)
    .def("remaining", &CliParser::remaining)
    .def("add_positional", &CliParser::add_positional)
    .def("set_program_description", &CliParser::set_program_description)
    .def("set_description", &CliParser::set_description)
    .def("set_required", &CliParser::set_required)
    .def("__getitem__", &CliParser::operator[])
    .def("has", &CliParser::has)
    // templated ones
    .def("add_flag",
         static_cast<void (CliParser::*)(std::string)>(
           &CliParser::add_flag))
    .def("add_flag",
         static_cast<void (CliParser::*)(std::string, std::string)>(
           &CliParser::add_flag))
    .def("add_flag",
         static_cast<void (CliParser::*)(std::string, std::string, std::string)>(
           &CliParser::add_flag))
    .def("add_argflag",
         static_cast<void (CliParser::*)(std::string)>(
           &CliParser::add_argflag))
    .def("add_argflag",
         static_cast<void (CliParser::*)(std::string, std::string)>(
           &CliParser::add_argflag))
    .def("add_argflag",
         static_cast<void (CliParser::*)(std::string, std::string, std::string)>(
           &CliParser::add_argflag))
    // overloaded ones
    .def("usage", py::overload_cast<>(&CliParser::usage))
    .def("usage", py::overload_cast<const std::string&>(&CliParser::usage))
    .def("parse", py::overload_cast<std::vector<std::string>>(&CliParser::parse))
    .def("parse_with_exceptions",
         py::overload_cast<std::vector<std::string>>(
           &CliParser::parse_with_exceptions))
    //.def("get", &CliParser::get)
    ;
}
