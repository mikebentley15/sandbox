CXX      := g++
TARGET   := stubbed
SOURCE   := $(wildcard *.cc)
OBJ      := $(SOURCE:%.cc=%.o)
CXXFLAGS :=
CXXFLAGS := -include exit.h
#CXXFLAGS += -Dexit=exit_stub

.PHONY: all clean distclean run
all: $(TARGET)
clean:
	rm -f $(OBJ)
distclean: clean
	rm -f $(TARGET)
run: $(TARGET)
	./$(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(LDFLAGS) -o $(TARGET) $(OBJ)

%.o: %.cc Makefile exit.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

