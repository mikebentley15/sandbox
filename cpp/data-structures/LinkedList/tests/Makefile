CC          := g++

CFLAGS      := -O2
CFLAGS      += -std=c++14
CFLAGS      += -I..
LDFLAGS     :=
SRC         := $(wildcard *.cpp)
EXTRA_SRC   := $(wildcard ../*.cpp)
HEADERS     := $(wildcard ../*.h)
TARGETS     := $(SRC:%.cpp=%)
RUN_TARGETS := $(TARGETS:%=run_%)
HARNESS     := test_harness.h

.PHONY: build check help clean run_%
build: $(TARGETS)

check: $(TARGETS) $(RUN_TARGETS)

help:
	@echo "Makefile for running tests on FLiT framework"
	@echo "  help     print this help documentation and exit"
	@echo "  build    just compile the targets"
	@echo "  check    run tests and print results to the console"
	@echo "  clean    remove all generated files"

clean:
	rm -f $(TARGETS)

run_% : %
	@./$< --quiet

% : %.cpp $(HARNESS) $(EXTRA_SRC) $(HEADERS) Makefile
	$(CC) $(CFLAGS) $< $(EXTRA_SRC) -o $@ $(LDFLAGS)

