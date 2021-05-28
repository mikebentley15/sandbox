SCAD_FILE       = needle-robot.scad
SCAD_INCLUDES   = helpers.scad
SCAD_INCLUDES  += screwlib.scad
MODELS         :=
#MODELS         += all.stl
#MODELS         += base.stl
#MODELS         += force-sensor.stl
#MODELS         += motor.stl
MODELS         += printed-1.stl
MODELS         += printed-2.stl
MODELS         += printed-3.stl
MODELS         += printed-4.stl
OPENSCAD_VARS   =

HOME            = $(PWD)

.PHONY: all
all: ${MODELS}

%.stl: ${SCAD_FILE} ${SCAD_INCLUDES} Makefile
	openscad -o $*.stl ${OPENSCAD_VARS} -D 'part="$*"' -D '$$fn=200' ${SCAD_FILE}

.PHONY: clean
clean:
	$(RM) ${MODELS}
