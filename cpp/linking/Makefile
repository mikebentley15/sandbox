CPP       := g++
CFLAGS    := -O0 --no-inline -g
LFLAGS    :=
SOURCES   := main.cpp
SOURCES   += foo.cpp
HEADERS   := foo.h
HEADERS   := foo2.h
OBJ       := $(SOURCES:%.cpp=%.o)

TARGET    := linking

.PHONY: all clean distclean
all: $(TARGET)
clean:
	rm -f $(OBJ)
distclean: clean
	rm -f $(TARGET)

%.o: %.cpp Makefile $(HEADERS)
	$(CPP) $(CFLAGS) -c -o $@ $<

$(TARGET): $(OBJ) Makefile
	$(CPP) $(CFLAGS) -o $(TARGET) $(OBJ) $(LFLAGS)
