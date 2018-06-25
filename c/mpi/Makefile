CC       := mpicc
SOURCES  := $(wildcard *.c)
TARGETS  := $(SOURCES:%.c=%)

PROCS    := 2
RUNNER   := mpirun -n $(PROCS)

.PHONY: all clean help run

all: $(TARGETS)

clean:
	rm -f $(TARGETS)

help:
	@echo "Available options are"
	@echo "  all:   compile all targets"
	@echo "  clean: remove compiled targets"
	@echo "  help:  show this documentation and exit"
	@echo "  run:   compile and run all targets"

run: $(TARGETS)
	@$(foreach t,$(TARGETS),echo "=== $t ===" && $(RUNNER) ./$t && echo;)

%: %.c
	$(CC) -o $@ $<

