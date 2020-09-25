CC       := gcc
CXX      := g++
CXXFLAGS := -O3

TARGETS := c_run cpp_run arraytypes flatten_multiarray

.PHONY: all
all: $(TARGETS)
$(TARGETS): Makefile

c_run: runner.c
	$(CC) $(CFLAGS) -o $@ $<

cpp_run: runner.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

arraytypes: arraytypes.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

flatten_multiarray: flatten_multiarray.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

.PHONY: clean
clean:
	rm -f $(TARGETS)
