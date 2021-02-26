SCAD_FILE       = cylinder-foot.scad
SCAD_INCLUDES   = helpers.scad
MODELS          = desk-foot.stl washing-machine-foot.stl

HOME            = $(PWD)

SVG            := $(wildcard *.svg)
DXF            := $(SVG:%.svg=%.dxf)
EPS            := $(SVG:%.svg=%.eps)

.PHONY: all
all: ${DXF} ${MODELS}

desk-foot.stl: OPENSCAD_VARS += -D base_height=2.0
desk-foot.stl: OPENSCAD_VARS += -D foot_radius=11.5
desk-foot.stl: OPENSCAD_VARS += -D wall_height=3.0
desk-foot.stl: OPENSCAD_VARS += -D wall_thickness=3.0

washing-machine-foot.stl: OPENSCAD_VARS += -D base_height=16.0
washing-machine-foot.stl: OPENSCAD_VARS += -D foot_radius=18.6
washing-machine-foot.stl: OPENSCAD_VARS += -D wall_height=8.0
washing-machine-foot.stl: OPENSCAD_VARS += -D wall_thickness=7.0

%.stl: ${SCAD_FILE} ${SCAD_INCLUDES}
	openscad -o $*.stl ${OPENSCAD_VARS} ${SCAD_FILE}


.PHONY: clean
clean:
	$(RM) ${MODELS} ${DXF} ${EPS}
