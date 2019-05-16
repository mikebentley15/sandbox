CC      := gcc
SOURCE  := $(wildcard *.c)
OBJ     := $(SOURCE:%.c=%.o)
TARGET  := alloc

.PHONY: all clean
all: $(TARGET)
clean:
	rm -f $(OBJ) $(TARGET)

$(TARGET): $(OBJ)
	$(CC) -o $(TARGET) $(OBJ)

%.o: %.c
	$(CC) -c -o $@ $<
