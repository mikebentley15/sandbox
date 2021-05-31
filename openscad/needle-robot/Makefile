SCAD_FILE      := needle-robot.scad
TEST_FILE      := prismatic-inner-test.scad
SCAD_INCLUDES  := helpers.scad
SCAD_INCLUDES  += screwlib.scad
MODELS         :=
MODELS         += motor-mount.stl
MODELS         += sensor-mount.stl
MODELS         += L-bind.stl
#MODELS         += bearing-mount.stl
MODELS         += motor-prismatic-coupler.stl
#MODELS         += needle-prismatic-coupler.stl
MODELS         += ${TEST_FILE:%.scad=%.stl}
OPENSCAD_VARS   = -D '$$fn=200'

HOME            = $(PWD)

.PHONY: all
all: ${MODELS}

prismatic-inner-test.stl: prismatic-inner-test.scad Makefile
	openscad -o $@ $<

%.stl: ${SCAD_FILE} ${SCAD_INCLUDES} Makefile
	openscad -o $*.stl ${OPENSCAD_VARS} -D 'part="$*"' ${SCAD_FILE}

.PHONY: clean
clean:
	$(RM) ${MODELS}
