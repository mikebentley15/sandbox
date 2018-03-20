GCC          := g++
CLANG        := clang++
LINK         := g++

CFLAGS       := -O3
CLANG_FLAGS  := $(CFLAGS)
CLANG_FLAGS  +=
GCC_FLAGS    := $(CFLAGS)
GCC_FLAGS    +=

LINK_FLAGS   :=

CLANG_SRC    := clang-src.cpp
GCC_SRC      := gcc-src.cpp
GCC_SRC      += main.cpp

OBJ          := $(CLANG_SRC:%.cpp=%_clang.o)
OBJ          += $(GCC_SRC:%.cpp=%_gcc.o)

TARGET       := mixed-compile


.PHONY: all clean
all: $(TARGET)
clean:
	rm -f $(TARGET)
	rm -f $(OBJ)

$(TARGET): $(OBJ)
	$(LINK) $(LINK_FLAGS) -o $(TARGET) $(OBJ)

%_clang.o: %.cpp Makefile
	$(CLANG) $(CLANG_FLAGS) -c -o $@ $<

%_gcc.o: %.cpp Makefile
	$(GCC) $(GCC_FLAGS) -c -o $@ $<

