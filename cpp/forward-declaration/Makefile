SOURCES  :=
SOURCES  += in-vector.cpp
SOURCES  += in-shared-ptr.cpp

TARGETS  := $(SOURCES:%.cpp=%)

all: $(TARGETS)
clean:
	$(RM) $(TARGETS)

%: %.cpp
	$(CXX) -std=c++20 -O3 -o $@ $<
