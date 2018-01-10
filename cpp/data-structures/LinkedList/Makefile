SUB_MAKES     := $(wildcard */Makefile */makefile)
SUB_DIRS      := $(patsubst %/,%,$(dir $(SUB_MAKES)))
CHECK_TARGETS := $(addprefix check__,$(SUB_DIRS))
CLEAN_TARGETS := $(addprefix clean__,$(SUB_DIRS))
BUILD_TARGETS := $(addprefix build__,$(SUB_DIRS))

.PHONY: build check help clean
help:
	@echo "Makefile for running tests on FLiT framework"
	@echo "  help     print this help documentation and exit"
	@echo "  build    just compile the $(TESTER), and all subdirectories"
	@echo "  check    run tests and print results to the console"
	@echo "  clean    remove all generated files"

build: $(TESTER) $(BUILD_TARGETS)

check: $(CHECK_TARGETS)
	@echo "All tests pass"

clean: $(CLEAN_TARGETS)

.PHONY: build__% check__% clean__%

build__%:
	@$(MAKE) build --directory $*

check__%:
	@$(MAKE) check --directory $*

clean__%:
	@$(MAKE) clean --directory $*

