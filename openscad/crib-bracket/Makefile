SCAD_FILE       = crib-bracket.scad
SCAD_INCLUDES   = helpers.scad
MODELS          = crib-bracket-R.stl crib-bracket-L.stl
OPENSCAD_VARS   =
DXF_SETTINGS    = "dxf_s: -polyaslines -splineaspolyline -splineprecision 20 -mm"

HOME            = $(PWD)

SVG            := $(wildcard *.svg)
DXF            := $(SVG:%.svg=%.dxf)
EPS            := $(SVG:%.svg=%.eps)

.PHONY: all
all: ${DXF} ${MODELS}

crib-bracket-R.stl: OPENSCAD_VARS=-D 'type="R"'
crib-bracket-L.stl: OPENSCAD_VARS=-D 'type="L"'

%.stl: ${SCAD_FILE} ${SCAD_INCLUDES}
	openscad -o $@ $(OPENSCAD_VARS) $<

%.eps: %.svg
	inkscape --export-area-page --export-eps="${HOME}/$*.eps" "${HOME}/$*.svg"

%.dxf: %.eps
	pstoedit  -f "${DXF_SETTINGS}" "$*.eps" "$*.dxf"


.PHONY: clean
clean:
	$(RM) ${MODELS} ${DXF} ${EPS}
