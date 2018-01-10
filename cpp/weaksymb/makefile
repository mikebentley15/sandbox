CLANG    = clang
GCC      = gcc

C_SRC    = foo.c
CPP_SRC  = foo.cpp

TARGETS += $(C_SRC:%.c=lib%-c-gcc.so)
TARGETS += $(C_SRC:%.c=lib%-c-gcc-direct.so)
TARGETS += $(C_SRC:%.c=lib%-c-clang.so)
TARGETS += $(C_SRC:%.c=lib%-c-clang-direct.so)
TARGETS += $(CPP_SRC:%.cpp=lib%-cpp-gcc.so)
TARGETS += $(CPP_SRC:%.cpp=lib%-cpp-gcc-direct.so)
TARGETS += $(CPP_SRC:%.cpp=lib%-cpp-clang.so)
TARGETS += $(CPP_SRC:%.cpp=lib%-cpp-clang-direct.so)

OBJ     += $(C_SRC:%.c=%-gcc-c.o)
OBJ     += $(C_SRC:%.c=%-clang-c.o)
OBJ     += $(CPP_SRC:%.cpp=%-gcc-cpp.o)
OBJ     += $(CPP_SRC:%.cpp=%-clang-cpp.o)

.PHONY: all
all: $(TARGETS)

.PHONY: clean
clean:
	rm -f $(OBJ)
	rm -f $(TARGETS)

lib%-c-gcc.so: %-gcc-c.o
	$(GCC) -shared -o $@ $<

lib%-cpp-gcc.so: %-gcc-cpp.o
	$(GCC) -shared -o $@ $<

lib%-c-gcc-direct.so: %.c
	$(GCC) -fPIC -shared -o $@ $<

lib%-cpp-gcc-direct.so: %.cpp
	$(GCC) -fPIC -shared -o $@ $<


lib%-c-clang.so: %-clang-c.o
	$(CLANG) -shared -o $@ $<

lib%-cpp-clang.so: %-clang-cpp.o
	$(CLANG) -shared -o $@ $<

lib%-c-clang-direct.so: %.c
	$(CLANG) -fPIC -shared -o $@ $<

lib%-cpp-clang-direct.so: %.cpp
	$(CLANG) -fPIC -shared -o $@ $<


%-gcc-c.o: %.c
	$(GCC) -fPIC -c -o $@ $<

%-gcc-cpp.o: %.cpp
	$(GCC) -fPIC -c -o $@ $<

%-clang-c.o: %.c
	$(CLANG) -fPIC -c -o $@ $<

%-clang-cpp.o: %.cpp
	$(CLANG) -fPIC -c -o $@ $<

