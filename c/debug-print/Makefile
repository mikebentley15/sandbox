CC        := gcc
CFLAGS    :=
LDFLAGS   :=
DEFINES   :=
CFLAGS    += $(DEFINES:%=-D%)
TARGET    := debugprint
SOURCES   := $(wildcard *.c)
HEADERS   := $(wildcard *.h)
OBJ       := $(SOURCES:%.c=%.o)

.PHONY: all clean check test
all: $(TARGET)
clean:
	rm -f $(OBJ)
	rm -f $(TARGET)
check: test
test: $(TARGET)
	./$(TARGET) test

$(TARGET): $(OBJ) Makefile
	$(CC) $(CFLAGS) $(OBJ) $(LDFLAGS) -o $(TARGET)

%.o: %.c $(HEADERS) Makefile
	$(CC) $(CFLAGS) -c -o $@ $<
