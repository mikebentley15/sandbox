CC            := gcc
CFLAGS        := -O2 -g
LDFLAGS       := -lelf
SOURCE        := $(wildcard *.c)
OBJ           := $(SOURCE:%.c=%.o)
TARGETS       := $(SOURCE:%.c=%)

# disable built-in rules
.SUFFIXES:

# disable deletion of intermediate files
.SECONDARY:

.PHONY: all clean
all: $(TARGETS)
clean:
	rm -f $(TARGETS)
	rm -f $(OBJ)

%: %.o
	$(CC) $(LDFLAGS) $< -o $@

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c $< -o $@
