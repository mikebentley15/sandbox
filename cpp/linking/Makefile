CPP       := g++
CFLAGS    := -O0 --no-inline -g -std=c++11
LFLAGS    :=
SOURCES   := main.cpp
SOURCES   += foo.cpp
HEADERS   := foo.h
HEADERS   := foo2.h
OBJ_01    := $(SOURCES:%.cpp=%-01.o)
OBJ_02    := $(SOURCES:%.cpp=%-02.o)
OBJ       := $(OBJ_01) $(OBJ_02)

TARGETS   :=
TARGETS   += linking01
TARGETS   += linking02

.PHONY: all clean distclean
all: $(TARGETS)
clean:
	rm -f $(OBJ)
distclean: clean
	rm -f $(TARGETS)

%-01.o: %.cpp Makefile $(HEADERS)
	$(CPP) $(CFLAGS) -c -o $@ $<

%-02.o: %-01.o Makefile
	objcopy -g --redefine-sym _Z12inline_hfuncv=$*_Z12inline_hfuncv $< $@

linking01: $(OBJ_01) Makefile
	$(CPP) $(CFLAGS) -o $@ $(OBJ_01) $(LFLAGS)

linking02: $(OBJ_02) Makefile
	$(CPP) $(CFLAGS) -o $@ $(OBJ_02) $(LFLAGS)


