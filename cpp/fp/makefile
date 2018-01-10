CC         = gcc
CFLAGS    += --std=c++11

SOURCES   += mantissa-test.cpp
TARGETS    = $(SOURCES:%.cpp=%)

.PHONY: all
all: $(TARGETS)

.PHONY: clean
clean:
	rm -f $(TARGETS)

%: %.cpp
	$(CC) $(CFLAGS) -o $@ $<
