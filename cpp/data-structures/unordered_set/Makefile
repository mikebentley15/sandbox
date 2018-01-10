CC          = clang++

CFLAGS     += -std=c++14
CFLAGS     += -g

HEADERS    += SimpleSet.h
HEADERS    += util.h

SOURCES    += main.cpp

TARGET      = set-timing

.PHONY: all
all: $(TARGET)

$(TARGET): $(SOURCES) $(HEADERS) Makefile
	$(CC) $(CFLAGS) $(SOURCES) -o $(TARGET)

.PHONY: clean
clean:
	rm -f $(TARGET)

.PNONY: run
run: $(TARGET)
	./$(TARGET)

