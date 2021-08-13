CC            := gcc
CFLAGS        := -O2 -g
SOURCE        := demo.c
SOURCE        += main.c
SOURCE        += minireadelf.c
SOURCE        += runtime-address.c
OBJ           := $(SOURCE:%.c=%.o)
TARGETS       := demo minireadelf runtime-address
ELF_TARG      := $(SOURCE:%.c=%.o-readelf.txt)
ELF_TARG      += $(TARGETS:%=%.exe-readelf.txt)
NM_TARG       := $(ELF_TARG:%-readelf.txt=%-nm.txt)
OBJDUMP_TARG  := $(ELF_TARG:%-readelf.txt=%-objdump.txt)

READELF       := readelf --all
NM            := nm
OBJDUMP       := objdump -D
OBJ2YAML      := $(lastword $(sort $(wildcard /usr/bin/obj2yaml*)))

.PHONY: all targets elfread nm objdump yaml clean
all: targets elfread nm objdump yaml
targets: $(TARGETS)
elfread: $(OBJ) $(ELF_TARG)
nm: $(OBJ) $(NM_TARG)
objdump: $(OBJ) $(OBJDUMP_TARG)
clean:
	$(RM) $(TARGETS)
	$(RM) $(OBJ)
	$(RM) $(ELF_TARG)
	$(RM) $(NM_TARG)
	$(RM) $(OBJDUMP_TARG)

demo: demo.o main.o
	$(CC) $(CFLAGS) demo.o main.o -o demo

%: %.o
	$(CC) $(CFLAGS) $< -o $@

%.o: %.c Makefile
	$(CC) $(CFLAGS) -c $< -o $@

%.o-readelf.txt: %.o
	$(READELF) $< > $@

%.exe-readelf.txt: %
	$(READELF) $< > $@

%.o-nm.txt: %.o
	$(NM) $< > $@

%.exe-nm.txt: %
	$(NM) $< > $@

%.o-objdump.txt: %.o
	$(OBJDUMP) $< > $@

%.exe-objdump.txt: %
	$(OBJDUMP) $< > $@

ifneq ($(OBJ2YAML),)

OBJ           := $(SOURCE:%.c=%.o)
YAML_OUTPUT   := $(OBJ:%.o=%.o-to.yaml)
YAML_BACK     := $(YAML_OUTPUT:%.yaml=%.yaml-to.o)
YAML2OBJ      := $(subst obj2yaml,yaml2obj,$(OBJ2YAML))

yaml: $(YAML_OUTPUT) $(YAML_BACK)

.PHONY: yaml-clean
clean: yaml-clean
yaml-clean:
	$(RM) $(YAML_OUTPUT)
	$(RM) $(YAML_BACK)

%.o-to.yaml: %.o
	$(OBJ2YAML) $< > $@

%.yaml-to.o: %.yaml
	$(YAML2OBJ) $< > $@

else

yaml:
	@echo "no obj2yaml executable found"

endif
