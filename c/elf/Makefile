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

.PHONY: all targets elfread nm objdump clean
all: targets elfread nm objdump
targets: $(TARGETS)
elfread: $(OBJ) $(ELF_TARG)
nm: $(OBJ) $(NM_TARG)
objdump: $(OBJ) $(OBJDUMP_TARG)
clean:
	rm -f $(TARGETS)
	rm -f $(OBJ)
	rm -f $(ELF_TARG)
	rm -f $(NM_TARG)
	rm -f $(OBJDUMP_TARG)

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
