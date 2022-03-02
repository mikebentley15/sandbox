TEST_HEADERS  := $(wildcard tst_*.h)
TARGETS       := $(TEST_HEADERS:%=run__%)

VERBOSE       ?= 0
ifeq ($(VERBOSE),0)
OUTPUT        := /dev/null
else
OUTPUT        := /dev/stdout
endif

.PHONY: check
check: $(TARGETS)

.PHONY: run__%.h
run__%.h: %.h
	c++ -E -o $(OUTPUT) $<
	@echo
