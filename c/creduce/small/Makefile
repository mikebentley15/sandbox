ORIGINAL    := small.c
TESTS       :=
TESTS       += test1.sh
TESTS       += test2.sh
TESTS       += test3.sh
RESULTS     := $(TESTS:test%.sh=small-%.c)

RM          := rm -f
CP          := cp
CREDUCE     := creduce

.PHONY: all clean
all: $(RESULTS)
clean:
	$(RM) $(RESULTS)
	$(RM) tmp*
	$(RM) a.out
	$(RM) $(RESULTS:%.c=%.o)

small-%.c: test%.sh $(ORIGINAL)
	@$(CP) $(ORIGINAL) $@
	$(CREDUCE) ./$< $@
	@$(RM) $@.orig

