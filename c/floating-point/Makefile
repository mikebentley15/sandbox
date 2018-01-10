CC      := gcc
CFLAGS  :=
LDFLAGS :=

SOURCES := float.c
SOURCES += main.c
OBJ     := $(SOURCES:%.c=%.o)

HEADERS := float.h

TARGET  := a.out

.PHONY: all clean distclean veryclean
all: $(TARGET)
clean:
	rm -f $(OBJ)
distclean: veryclean
veryclean: clean
	rm -f $(TARGET)

$(TARGET): $(OBJ) Makefile
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(TARGET) $(OBJ)

%.o: %.c $(HEADERS) Makefile
	$(CC) $(CFLAGS) -c -o $@ $<
