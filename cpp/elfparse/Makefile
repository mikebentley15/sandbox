CXX       = g++
CXXFLAGS  = -g -O3 -fPIC
SOURCE    = elffuncs.cpp
OBJ       = elffuncs.o
TARGET    = elffuncs

.PHONY: all clean
all: $(TARGET)
clean:
	rm -f $(OBJ)
	rm -f $(TARGET)

$(OBJ): $(SOURCE) Makefile
	$(CXX) -c $(CXXFLAGS) $(SOURCE) -o $(OBJ)

$(TARGET): $(OBJ) Makefile
	$(CXX) $(CXXFLAGS) $(OBJ) -o $(TARGET)
