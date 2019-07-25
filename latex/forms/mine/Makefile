TOPLEVEL   := $(wildcard *.tex)
TARGETS    := $(TOPLEVEL:%.tex=%.pdf)
MAKEFILES  := Makefile

DEPFILES   :=
DEPFILES   += $(wildcard *.cls)
DEPFILES   += $(wildcard *.bib)
DEPFILES   += $(wildcard *.bst)
DEPFILES   += $(wildcard inputs/*.tex)
DEPFILES   += $(wildcard inputs/algorithms/*.tex)
DEPFILES   += $(wildcard inputs/figures/*.tex)
DEPFILES   += $(wildcard inputs/tables/*.tex)
DEPFILES   += $(wildcard inputs/notes/*.tex)
DEPFILES   += $(wildcard images/*.pdf)
DEPFILES   += $(wildcard images/timing/*.pdf)
DEPFILES   += $(TOPLEVEL)
DEPFILES   += $(MAKEFILES)

GARBAGE    :=
GARBAGE    += comment.cut
GARBAGE    += $(TOPLEVEL:%.tex=%.log)
GARBAGE    += $(TOPLEVEL:%.tex=%.out)
GARBAGE    += $(TOPLEVEL:%.tex=%.aux)
GARBAGE    += $(TOPLEVEL:%.tex=%.dvi)
GARBAGE    += $(TOPLEVEL:%.tex=%.cb)
GARBAGE    += $(TOPLEVEL:%.tex=%.cb2)
GARBAGE    += $(TOPLEVEL:%.tex=%.bbl)
GARBAGE    += $(TOPLEVEL:%.tex=%.blg)
GARBAGE    += $(TOPLEVEL:%.tex=%.tdo)
GARBAGE    += $(TOPLEVEL:%.tex=%.markdown.in)
GARBAGE    += $(TOPLEVEL:%.tex=%.markdown.lua)
GARBAGE    += $(TOPLEVEL:%.tex=%.markdown.out)
GARBAGE    += $(addprefix _markdown_,$(TOPLEVEL:%.tex=%))
GARBAGE    += $(addprefix _minted-,$(TOPLEVEL:%.tex=%))
GARBAGE    += $(wildcard figures/*-eps-converted-to.pdf)

# To use custom fonts, build with xelatex instead
LATEX      := pdflatex
LATEX      += -shell-escape

.PHONY: all
all: $(TARGETS)

$(TARGET): $(DEPFILES)

ECHO            := echo

# empty means it works, nonempty means it doesn't work
ESCAPE_WORKS    := $(shell $(ECHO) -e)
NEWLINE_WORKS   := $(shell $(ECHO) -n)

# A Makefile function
# @param 1: color (e.g., BLUE or GREEN)
# @param 2: message to be printed in the color
color_out_noline = \
             if [ -t 1 ] && [ -z "$(ESCAPE_WORKS)" ] && \
                [ -z "$(NEWLINE_WORKS)" ]; then \
               $(ECHO) -n -e "$(BASH_$1)$2$(BASH_CLEAR)"; \
             elif [ -z "$(NEWLINE_WORKS)" ]; then \
               $(ECHO) -n "$2"; \
             else \
               $(ECHO) "$2"; \
             fi
color_out = \
             if [ -t 1 ] && [ -z "$(ESCAPE_WORKS)" ]; then \
               $(ECHO) -e "$(BASH_$1)$2$(BASH_CLEAR)"; \
             else \
               $(ECHO) "$2"; \
             fi

BASH_CLEAR    := \e[0m
BASH_BLACK    := \e[0;30m
BASH_BROWN    := \e[0;33m
BASH_GRAY     := \e[0;37m
BASH_GREY     := \e[0;37m
BASH_DARKGRAY := \e[1;30m
BASH_DARKGREY := \e[1;30m
BASH_RED      := \e[1;31m
BASH_GREEN    := \e[1;32m
BASH_YELLOW   := \e[1;33m
BASH_BLUE     := \e[1;34m
BASH_PURPLE   := \e[1;35m
BASH_CYAN     := \e[1;36m
BASH_WHITE    := \e[1;37m

# print statistics of a tex file compilation
# @param $1: name of the log file
define TEX_STATS
	echo "$1 statistics:"
	echo "    ----"
	$(call color_out_noline,BLUE,  overfull:               )
	grep "^Overfull" $1 | wc -l
	$(call color_out_noline,BLUE,  underfull:              )
	grep "^Underfull" $1 | wc -l
	echo "    ----"
	$(call color_out_noline,BLUE,  total warnings:         )
	grep -i warning $1 | wc -l
	$(call color_out_noline,BLUE,  undefined citations:    )
	grep "Warning: Citation .* undefined" $1 | wc -l
	$(call color_out_noline,BLUE,  undefined references:   )
	grep "Warning: Reference .* undefined" $1 | wc -l
	$(call color_out_noline,BLUE,  duplicate labels:       )
	grep "Warning: Label .* multiply defined" $1 | wc -l
	echo "    ----"
	if [ "$$(type -P detex)" ]; then \
		$(call color_out_noline,BLUE,  total word count:       ); \
		detex $(1:%.log=%.tex) | wc -w; \
	  echo "    ----"; \
	fi
endef

%.pdf: %.tex
	@echo "$< -> $@"
	@$(LATEX) $< </dev/null >/dev/null || (tail -n 30 $*.log && false)
	@$(LATEX) $< </dev/null >/dev/null
	@$(call TEX_STATS,$*.log)

images/%.eps: images/%.dot
	dot $< -Tps -o $@

images/%.ps: images/%.tex
	@echo "$< -> $@"
	@(cd images && $(LATEX) $(notdir $<)) </dev/null >/dev/null \
		|| (tail -n 30 $(<:%.tex=%.log) && false)

.PHONY: clean
clean:
	rm -rf $(GARBAGE)

.PHONY: veryclean
veryclean: clean
	rm -f $(TARGETS)

.PHONY: distclean
distclean: veryclean
