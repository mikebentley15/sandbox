SCAD_FILE       = sam-logo-stamp.scad
DEPENDENCIES   += logo-small.svg
MODELS         += ${SCAD_FILE:%.scad=%.stl}
OPENSCAD_VARS   =

.PHONY: all
all: ${MODELS}

${MODELS}: ${SCAD_FILE} ${DEPENDENCIES}
	openscad -o $@ ${OPENSCAD_VARS} $<

.PHONY: clean
clean:
	$(RM) ${MODELS}
