DEPENDENCIES  := $(wildcard inputs/*.tex)
DEPENDENCIES  += $(wildcard images/*.jpg)
DEPENDENCIES  += $(wildcard images/*.png)

GARBAGE       := main.log
GARBAGE       += main.aux
GARBAGE       += main.out

.PHONY: default all clean veryclean distclean
default: all
all: main.pdf
clean:
	rm -f $(GARBAGE)
veryclean: distclean
distclean: clean
	rm -f main.pdf

main.pdf: main.tex ${DEPENDENCIES}
	pdflatex main.tex </dev/null

