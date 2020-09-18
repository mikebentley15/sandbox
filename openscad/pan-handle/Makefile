SCAD_FILE       = pan-handle.scad
SCAD_INCLUDES   = helpers.scad
MODELS          = pan-handle.stl
OPENSCAD_VARS   =
DXF_SETTINGS    = "dxf_s: -polyaslines -splineaspolyline -splineprecision 20 -mm"

HOME            = $(PWD)

SVG            := $(wildcard *.svg)
DXF            := $(SVG:%.svg=%.dxf)
EPS            := $(SVG:%.svg=%.eps)

.PHONY: all
all: ${DXF} ${MODELS}


%.stl: ${SCAD_FILE} ${SCAD_INCLUDES}
	openscad -o "$@" ${OPENSCAD_VARS} "$<"

%.eps: %.svg
	inkscape --export-area-page --export-eps="$@" "$<"

%.dxf: %.eps
	pstoedit  -f "${DXF_SETTINGS}" "$<" "$@"


.PHONY: clean
clean:
	$(RM) ${MODELS} ${DXF} ${EPS}
