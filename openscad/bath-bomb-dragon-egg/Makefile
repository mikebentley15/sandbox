SCAD_FILE       = bath-bomb-dragon-egg.scad
SCAD_INCLUDES   = helpers.scad
DEP_STL         = Dragon_Egg_Mi_solid-fixed-18k.stl
MODELS          = top.stl bottom.stl shell.stl

HOME            = $(PWD)

.PHONY: all
all: ${MODELS}

%.stl: ${SCAD_FILE} ${SCAD_INCLUDES}
	openscad -o $*.stl ${OPENSCAD_VARS} -D 'part="$*"' ${SCAD_FILE}

.PHONY: clean
clean:
	$(RM) ${MODELS}
