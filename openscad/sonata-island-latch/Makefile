SCAD_FILE       = sonata-island-latch.scad
SCAD_INCLUDES   = helpers.scad
MODELS          = $(SCAD_FILE:%.scad=%.stl)
OPENSCAD_VARS   =
DXF_SETTINGS    = "dxf_s: -polyaslines -splineaspolyline -splineprecision 20 -mm"

HOME            = $(PWD)

SVG            := $(wildcard *.svg)
DXF            := $(SVG:%.svg=%.dxf)
EPS            := $(SVG:%.svg=%.eps)

.PHONY: all
all: ${DXF} ${MODELS}


%.stl: ${SCAD_FILE} ${SCAD_INCLUDES}
	openscad -o $*.stl ${OPENSCAD_VARS} ${SCAD_FILE}

%.eps: %.svg
	inkscape --export-area-page --export-eps="${HOME}/$*.eps" "${HOME}/$*.svg"

%.dxf: %.eps
	pstoedit  -f "${DXF_SETTINGS}" "$*.eps" "$*.dxf"


.PHONY: clean
clean:
	$(RM) ${MODELS} ${DXF} ${EPS}
