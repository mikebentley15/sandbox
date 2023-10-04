SOURCES   := interface.cpp
HEADERS   :=
CXXFLAGS  += -O3 -std=c++20
TARGETS   := $(SOURCES:%.cpp=%)

.PHONY: all default clean
default: all
all: $(TARGETS)
clean:
	$(RM) $(TARGETS)

%: %.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $<
