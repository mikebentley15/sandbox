SCAD_FILE       = screws.scad
SCAD_INCLUDES   = helpers.scad screwlib.scad
MODELS          = screws.stl
OPENSCAD_VARS   = -D 'segment=3'

.PHONY: all
all: ${MODELS}

%.stl: ${SCAD_FILE} ${SCAD_INCLUDES}
	openscad -o $*.stl ${SCAD_FILE}

.PHONY: clean
clean:
	$(RM) ${MODELS}
