SOURCE  := linspace.cpp
HEADERS := PairWise.hpp
HEADERS += Chunk.hpp
TARGET  := $(SOURCE:%.cpp=%)
CXXFLAGS += -std=c++20 -O3

.PHONY: default all clean
default: all
all: $(TARGET)
clean:
	$(RM) $(TARGET)

%: %.cpp PairWise.hpp
	$(CXX) $(CXXFLAGS) $< -o $@
