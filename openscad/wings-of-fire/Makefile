SCAD_FILES     += wings-of-fire-cake-topper.scad
SCAD_FILES     += wings-of-fire-coin.scad
SCAD_FILES     += wings-of-fire-stamp.scad
DEPENDENCIES   += helpers.scad
DEPENDENCIES   += wings-of-fire.svg
MODELS         += ${SCAD_FILES:%.scad=%.stl}
OPENSCAD_VARS   =

.PHONY: all
all: ${MODELS}

%.stl: %.scad ${DEPENDENCIES}
	openscad -o $@ ${OPENSCAD_VARS} $<

.PHONY: clean
clean:
	$(RM) ${MODELS}
