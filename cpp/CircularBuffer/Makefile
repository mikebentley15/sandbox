TARGETS  :=
TARGETS  += main
TARGETS  += tst_CircularBuffer
CXXFLAGS += -Wall -Wextra -Wpedantic -Werror
CXXFLAGS += -std=c++20

.PHONY: default clean
default: $(TARGETS)
clean:
	rm -f $(TARGETS)

main: main.cpp CircularBuffer.h
	$(CXX) $(CXXFLAGS) $< -o $@

tst_CircularBuffer: tst_CircularBuffer.cpp CircularBuffer.h catch.hpp
	$(CXX) $(CXXFLAGS) $< -o $@
