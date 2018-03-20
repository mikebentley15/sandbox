GCC          := g++
CLANG        := clang++
#LINK         := g++
LINK         := clang++ -v
#LINK         := /usr/bin/ld

CFLAGS       := -O3 # -fpie
CLANG_FLAGS  := $(CFLAGS)
CLANG_FLAGS  +=
GCC_FLAGS    := $(CFLAGS)
GCC_FLAGS    +=

LINK_FLAGS   := -pie

# Good link flags from Clang
#LINK_FLAGS   += --eh-frame-hdr
#LINK_FLAGS   += -m
#LINK_FLAGS   += elf_x86_64
#LINK_FLAGS   += -dynamic-linker
#LINK_FLAGS   += /lib64/ld-linux-x86-64.so.2
#LINK_FLAGS   += /usr/lib64/gcc/x86_64-pc-linux-gnu/7.3.0/../../../../lib64/crt1.o
#LINK_FLAGS   += /usr/lib64/gcc/x86_64-pc-linux-gnu/7.3.0/../../../../lib64/crti.o
#LINK_FLAGS   += /usr/lib64/gcc/x86_64-pc-linux-gnu/7.3.0/crtbegin.o
#LINK_FLAGS   += -L/usr/lib64/gcc/x86_64-pc-linux-gnu/7.3.0
#LINK_FLAGS   += -L/usr/lib64/gcc/x86_64-pc-linux-gnu/7.3.0/../../../../lib64
#LINK_FLAGS   += -L/lib/../lib64
#LINK_FLAGS   += -L/usr/lib/../lib64
#LINK_FLAGS   += -L/usr/lib64/gcc/x86_64-pc-linux-gnu/7.3.0/../../..
#LINK_FLAGS   += -L/opt/cling/bin/../lib
#LINK_FLAGS   += -L/lib
#LINK_FLAGS   += -L/usr/lib
#LINK_FLAGS   += -lstdc++
#LINK_FLAGS   += -lm
#LINK_FLAGS   += -lgcc_s
#LINK_FLAGS   += -lgcc
#LINK_FLAGS   += -lc
#LINK_FLAGS   += -lgcc_s
#LINK_FLAGS   += -lgcc
#LINK_FLAGS   += /usr/lib64/gcc/x86_64-pc-linux-gnu/7.3.0/crtend.o
#LINK_FLAGS   += /usr/lib64/gcc/x86_64-pc-linux-gnu/7.3.0/../../../../lib64/crtn.o

# Bad link flags from GCC
#LINK_FLAGS   += -plugin
#LINK_FLAGS   += /usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/liblto_plugin.so
#LINK_FLAGS   += -plugin-opt=/usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/lto-wrapper
#LINK_FLAGS   += -plugin-opt=-fresolution=/tmp/ccsrnsir.res
#LINK_FLAGS   += -plugin-opt=-pass-through=-lgcc_s
#LINK_FLAGS   += -plugin-opt=-pass-through=-lgcc
#LINK_FLAGS   += -plugin-opt=-pass-through=-lc
#LINK_FLAGS   += -plugin-opt=-pass-through=-lgcc_s
#LINK_FLAGS   += -plugin-opt=-pass-through=-lgcc
#LINK_FLAGS   += --build-id
#LINK_FLAGS   += --eh-frame-hdr
#LINK_FLAGS   += --hash-style=gnu
#LINK_FLAGS   += -m
#LINK_FLAGS   += elf_x86_64
#LINK_FLAGS   += -dynamic-linker
#LINK_FLAGS   += /lib64/ld-linux-x86-64.so.2
#LINK_FLAGS   += -pie
#LINK_FLAGS   += /usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/../../../../lib/Scrt1.o
#LINK_FLAGS   += /usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/../../../../lib/crti.o
#LINK_FLAGS   += /usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/crtbeginS.o
#LINK_FLAGS   += -L/usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0
#LINK_FLAGS   += -L/usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/../../../../lib
#LINK_FLAGS   += -L/lib/../lib
#LINK_FLAGS   += -L/usr/lib/../lib
#LINK_FLAGS   += -L/usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/../../..
#LINK_FLAGS   += -lstdc++
#LINK_FLAGS   += -lm
#LINK_FLAGS   += -lgcc_s
#LINK_FLAGS   += -lgcc
#LINK_FLAGS   += -lc
#LINK_FLAGS   += -lgcc_s
#LINK_FLAGS   += -lgcc
#LINK_FLAGS   += /usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/crtendS.o
#LINK_FLAGS   += /usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/../../../../lib/crtn.o

# Try to fix the GCC flags below
#LINK_FLAGS   += -plugin
#LINK_FLAGS   += /usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/liblto_plugin.so
#LINK_FLAGS   += -plugin-opt=/usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/lto-wrapper
#LINK_FLAGS   += -plugin-opt=-fresolution=/tmp/ccsrnsir.res
#LINK_FLAGS   += -plugin-opt=-pass-through=-lgcc_s
#LINK_FLAGS   += -plugin-opt=-pass-through=-lgcc
#LINK_FLAGS   += -plugin-opt=-pass-through=-lc
#LINK_FLAGS   += -plugin-opt=-pass-through=-lgcc_s
#LINK_FLAGS   += -plugin-opt=-pass-through=-lgcc
#LINK_FLAGS   += --build-id
#LINK_FLAGS   += --eh-frame-hdr
#LINK_FLAGS   += --hash-style=gnu
#LINK_FLAGS   += -m
#LINK_FLAGS   += elf_x86_64
#LINK_FLAGS   += -dynamic-linker
#LINK_FLAGS   += /lib64/ld-linux-x86-64.so.2
##LINK_FLAGS   += -pie
#LINK_FLAGS   += /usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/../../../../lib/Scrt1.o
#LINK_FLAGS   += /usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/../../../../lib/crti.o
#LINK_FLAGS   += /usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/crtbeginS.o
#LINK_FLAGS   += -L/usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0
#LINK_FLAGS   += -L/usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/../../../../lib
#LINK_FLAGS   += -L/lib/../lib
#LINK_FLAGS   += -L/usr/lib/../lib
#LINK_FLAGS   += -L/usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/../../..
#LINK_FLAGS   += -lstdc++
#LINK_FLAGS   += -lm
#LINK_FLAGS   += -lgcc_s
#LINK_FLAGS   += -lgcc
#LINK_FLAGS   += -lc
#LINK_FLAGS   += -lgcc_s
#LINK_FLAGS   += -lgcc
#LINK_FLAGS   += /usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/crtendS.o
#LINK_FLAGS   += /usr/lib/gcc/x86_64-pc-linux-gnu/7.3.0/../../../../lib/crtn.o


# It appears that mixing works, but for GCC to compile it, the main object file
# must have been compiled with GCC.
# Is the reverse true when linking with CLANG? NO!
# TODO: Why does GCC linker need to have main compiled with GCC?
# TODO: find out how to investigate the different linking commands by GCC and CLANG
# TODO: find out hidden flags from GCC and CLANG when compiling object files
CLANG_SRC    :=
GCC_SRC      :=
GCC_SRC      += clang-src.cpp
GCC_SRC      += gcc-src.cpp
#GCC_SRC      += main.cpp
#CLANG_SRC    += clang-src.cpp
#CLANG_SRC    += gcc-src.cpp
CLANG_SRC    += main.cpp

OBJ          := $(CLANG_SRC:%.cpp=%_clang.o)
OBJ          += $(GCC_SRC:%.cpp=%_gcc.o)

TARGET       := mixed-compile


.PHONY: all clean
all: $(TARGET)
clean:
	rm -f $(TARGET)
	rm -f $(OBJ)

$(TARGET): $(OBJ)
	$(LINK) -o $(TARGET) $(OBJ) $(LINK_FLAGS)

%_clang.o: %.cpp Makefile
	$(CLANG) $(CLANG_FLAGS) -c -o $@ $<

%_gcc.o: %.cpp Makefile
	$(GCC) $(GCC_FLAGS) -c -o $@ $<

