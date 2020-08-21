SCAD_FILE       = ant-trap.scad
SCAD_INCLUDES   = helpers.scad
MODELS          = $(SCAD_FILE:%.scad=%-top.stl)
MODELS         += $(SCAD_FILE:%.scad=%-bottom.stl)
OPENSCAD_VARS   =
DXF_SETTINGS    = "dxf_s: -polyaslines -splineaspolyline -splineprecision 20 -mm"

HOME            = $(PWD)

SVG            := $(wildcard *.svg)
DXF            := $(SVG:%.svg=%.dxf)
EPS            := $(SVG:%.svg=%.eps)

.PHONY: all
all: ${DXF} ${MODELS}

%-top.stl: OPENSCAD_VARS+=-D 'part="top"'
%-bottom.stl: OPENSCAD_VARS+=-D 'part="bottom"'

%.stl: ${SCAD_FILE} ${SCAD_INCLUDES} Makefile
	openscad -o $*.stl ${OPENSCAD_VARS} ${SCAD_FILE}

%.eps: %.svg Makefile
	inkscape --export-area-page --export-eps="${HOME}/$*.eps" "${HOME}/$*.svg"

%.dxf: %.eps Makefile
	pstoedit  -f "${DXF_SETTINGS}" "$*.eps" "$*.dxf"


.PHONY: clean
clean:
	$(RM) ${MODELS} ${DXF} ${EPS}
