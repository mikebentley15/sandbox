SOURCE  := linspace.cpp
TARGET  := $(SOURCE:%.cpp=%)
CXXFLAGS += -std=c++20 -O3

.PHONY: default all clean
default: all
all: $(TARGET)
clean:
	$(RM) $(TARGET)

%: %.cpp
	$(CXX) $(CXXFLAGS) $< -o $@
