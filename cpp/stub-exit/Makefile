CXX      := g++
TARGET   := stubbed
SOURCE   := $(wildcard *.cc)
OBJ      := $(SOURCE:%.cc=%.o)

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

%.o: %.cc Makefile
	$(CXX) $(CXXFLAGS) -c -o $@ $<

