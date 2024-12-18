// Autogenerated by gen_pybind11.py
#include <A.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>
#include <utility>

namespace bindings {

namespace {

//using ::A_ns::operator<<;

//template <typename T>
//std::string streamToString(const T &toStream) {
//    std::ostringstream out;
//    out << toStream;
//    return out.str();
//}

void bindA(pybind11::module& m) {
    auto classA = pybind11::class_<A_ns::A, std::shared_ptr<A_ns::A>>(m, "A", "");
    classA.def(pybind11::init([](){ return new A_ns::A(); }));
    classA.def(
        pybind11::init([](double a, std::string b, std::vector<A_ns::Point> points){
            return new A_ns::A(std::move(a), std::move(b), std::move(points));
        }),
        pybind11::arg("a") = 0.0,
        pybind11::arg("b") = "",
        pybind11::arg("points") = std::vector<A_ns::Point>{}
        );
    classA.def_readwrite("a", &A_ns::A::a);
    classA.def_readwrite("b", &A_ns::A::b);
    classA.def_readwrite("points", &A_ns::A::points);
}

void bindPoint(pybind11::module& m) {
    auto classPoint = pybind11::class_<A_ns::Point, std::shared_ptr<A_ns::Point>>(m, "Point", "");
    classPoint.def(pybind11::init([](){ return new A_ns::Point(); }));
    classPoint.def_readwrite("x", &A_ns::Point::x);
    classPoint.def_readwrite("y", &A_ns::Point::y);
    classPoint.def_readwrite("z", &A_ns::Point::z);
}

} // unnamed namespace

PYBIND11_MODULE(mystructs, module) {
    module.doc() = "mystructs module";
    auto submodule_A_ns = module.def_submodule("A_ns", "The A_ns namespace");

    bindA(submodule_A_ns);
    bindPoint(submodule_A_ns);
}


} // namespace bindings
