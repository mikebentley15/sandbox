SCAD_FILE       = screws.scad
OTHER_SCAD      = screw_thread_spool.scad
SCAD_INCLUDES   = helpers.scad screwlib.scad
MODELS          = screws.stl
OPENSCAD_VARS   = -D 'segment=3'

.PHONY: all
all: ${MODELS}

screws.stl: ${SCAD_FILE} ${SCAD_INCLUDES}
	openscad -o $*.stl ${SCAD_FILE}

my-screw.stl: screw_thread_spool.scad screwlib.scad Makefile
	openscad -D 'piece="screw"' -o $@ $<

my-nut.stl: screw_thread_spool.scad screwlib.scad Makefile
	openscad -D 'piece="nut"' -o $@ $<

.PHONY: clean
clean:
	$(RM) ${MODELS}
