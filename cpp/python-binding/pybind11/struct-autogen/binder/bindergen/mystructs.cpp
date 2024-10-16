// File: A.cpp
#include <A.hpp>
#include <sstream> // __str__

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_A(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // A_ns::Point file:A.hpp line:14
		pybind11::class_<A_ns::Point, std::shared_ptr<A_ns::Point>> cl(M("A_ns"), "Point", "Represents a 3D point\n\n \n x-coordinate\n \n\n y-coordinate\n \n\n z-coordinate");
		cl.def( pybind11::init( [](){ return new A_ns::Point(); } ) );
		cl.def_readwrite("x", &A_ns::Point::x);
		cl.def_readwrite("y", &A_ns::Point::y);
		cl.def_readwrite("z", &A_ns::Point::z);
	}
	{ // A_ns::A file:A.hpp line:20
		pybind11::class_<A_ns::A, std::shared_ptr<A_ns::A>> cl(M("A_ns"), "A", "");
		cl.def( pybind11::init( [](){ return new A_ns::A(); } ) );
		cl.def( pybind11::init( [](A_ns::A const &o){ return new A_ns::A(o); } ) );
		cl.def_readwrite("a", &A_ns::A::a);
		cl.def_readwrite("b", &A_ns::A::b);
		cl.def_readwrite("points", &A_ns::A::points);
	}
}


#include <map>
#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <pybind11/pybind11.h>

typedef std::function< pybind11::module & (std::string const &) > ModuleGetter;

void bind_A(std::function< pybind11::module &(std::string const &namespace_) > &M);


PYBIND11_MODULE(mystructs, root_module) {
	root_module.doc() = "mystructs module";

	std::map <std::string, pybind11::module> modules;
	ModuleGetter M = [&](std::string const &namespace_) -> pybind11::module & {
		auto it = modules.find(namespace_);
		if( it == modules.end() ) throw std::runtime_error("Attempt to access pybind11::module for namespace " + namespace_ + " before it was created!!!");
		return it->second;
	};

	modules[""] = root_module;

	static std::vector<std::string> const reserved_python_words {"nonlocal", "global", };

	auto mangle_namespace_name(
		[](std::string const &ns) -> std::string {
			if ( std::find(reserved_python_words.begin(), reserved_python_words.end(), ns) == reserved_python_words.end() ) return ns;
			else return ns+'_';
		}
	);

	std::vector< std::pair<std::string, std::string> > sub_modules {
		{"", "A_ns"},
	};
	for(auto &p : sub_modules ) modules[p.first.size() ? p.first+"::"+p.second : p.second] = modules[p.first].def_submodule( mangle_namespace_name(p.second).c_str(), ("Bindings for " + p.first + "::" + p.second + " namespace").c_str() );

	//pybind11::class_<std::shared_ptr<void>>(M(""), "_encapsulated_data_");

	bind_A(M);

}

// Source list file: /home/michael.bentley/git/sandbox/cpp/python-binding/pybind11/struct-autogen/binder/mystructs.sources
// mystructs.cpp
// A.cpp

// Modules list file: /home/michael.bentley/git/sandbox/cpp/python-binding/pybind11/struct-autogen/binder/mystructs.modules
// A_ns 
