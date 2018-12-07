CXX        = clang++
CXXFLAGS   = -g -O0 -std=c++11 -Wall -Wextra -Werror
LDFLAGS    =
SOURCE     = cpp-hier-parser.cpp
TARGET     = $(SOURCE:%.cpp=%)

.PHONY: all clean
all: $(TARGET)
clean:
	rm -f $(TARGET)

$(TARGET): $(SOURCE) Makefile
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE) $(LDFLAGS)
