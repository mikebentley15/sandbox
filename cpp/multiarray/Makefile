CC     = gcc
CXX    = g++

TARGETS := c_run cpp_run

.PHONY: all
all: $(TARGETS)

c_run: runner.c
	$(CC) -o $@ $<

cpp_run: runner.cpp
	$(CXX) -o $@ $<

.PHONY: clean
clean:
	rm -f $(TARGETS)
