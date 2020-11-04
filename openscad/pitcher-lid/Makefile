SCAD_FILE       = pitcher-lid.scad
MODEL           = $(SCAD_FILE:%.scad=%.stl)
OPENSCAD_VARS   = -D 'segment=3'
DXF_SETTINGS    = "dxf_s: -polyaslines -splineaspolyline -splineprecision 20 -mm"

HOME            = $(PWD)

SVG            := $(wildcard *.svg)
DXF            := $(SVG:%.svg=%.dxf)
EPS            := $(SVG:%.svg=%.eps)

.PHONY: all
all: ${MODEL}


${MODEL}: ${SCAD_FILE}
	openscad -o ${MODEL} ${OPENSCAD_VARS} ${SCAD_FILE}

%.eps: %.svg
	inkscape --export-area-page --export-eps="${HOME}/$*.eps" "${HOME}/$*.svg"

%.dxf: %.eps
	pstoedit  -f "${DXF_SETTINGS}" "$*.eps" "$*.dxf"


.PHONY: clean
clean:
	$(RM) ${MODELS} ${DXF} ${EPS}
