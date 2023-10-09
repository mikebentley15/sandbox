SOURCES  := tuple.cpp
HEADERS  := tuple.hpp
TARGETS  := $(SOURCES:%.cpp=%)
CXXFLAGS := -std=c++17 -O3

.PHONY: default all clean
default: all
all: $(TARGETS)
clean:
	$(RM) $(TARGETS)

%: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $< -o $@
