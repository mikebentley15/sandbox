SOURCES   := signals.cpp
HEADERS   := DirectSignal.hpp
HEADERS   := SignalConnector.hpp
HEADERS   := SignalBlocker.hpp
HEADERS   := SignalMapper.hpp
TARGETS   := $(SOURCES:%.cpp=%)

CXXFLAGS  := -O3 -std=c++20

.PHONY: all default clean
default: all
all: $(TARGETS)
clean:
	$(RM) $(TARGETS)

%: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $< -o $@
