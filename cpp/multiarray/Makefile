CC     = gcc
CXX    = g++

TARGETS := c_run cpp_run arraytypes

.PHONY: all
all: $(TARGETS)

c_run: runner.c
	$(CC) -o $@ $<

cpp_run: runner.cpp
	$(CXX) -o $@ $<

arraytypes: arraytypes.cpp
	$(CXX) -o $@ $<

.PHONY: clean
clean:
	rm -f $(TARGETS)
