COMMA    := ,

MYVAR    := -Wl,hello -Wl,hi-there -other-flag
MYNEWVAR := $(patsubst -Wl$(COMMA)%,-Xlinker=%,$(MYVAR))

.PHONY: print noprint
print:
	$(info MYVAR    = $(MYVAR))
	$(info MYNEWVAR = $(MYNEWVAR))
	@true

noprint:
