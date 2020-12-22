SCAD_FILE      := cookie-cutter-dala-horse.scad
DEPENDENCIES   := helpers.scad
DEPENDENCIES   += outline.svg
DEPENDENCIES   += decals.svg
DEPENDENCIES   += filled.svg
MODELS         := cookie-cutter-dala-horse-5.4in.stl
MODELS         += cookie-cutter-dala-horse-4in.stl
MODELS         += cookie-cutter-dala-horse-3in.stl
OPENSCAD_VARS  :=

.PHONY: all
all: ${MODELS}


cookie-cutter-dala-horse-5.4in.stl: ${SCAD_FILE} ${DEPENDENCIES}
	openscad -o $@ ${OPENSCAD_VARS} ${SCAD_FILE} -D svg_scale=1.0

cookie-cutter-dala-horse-4in.stl: ${SCAD_FILE} ${DEPENDENCIES}
	openscad -o $@ ${OPENSCAD_VARS} ${SCAD_FILE} -D svg_scale=0.74

cookie-cutter-dala-horse-3in.stl: ${SCAD_FILE} ${DEPENDENCIES}
	openscad -o $@ ${OPENSCAD_VARS} ${SCAD_FILE} -D svg_scale=0.555

.PHONY: clean
clean:
	$(RM) ${MODELS}
