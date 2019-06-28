CC          = clang++

CFLAGS     += -std=c++14
CFLAGS     += -g

HEADERS    +=

SOURCES    += null-reference.cpp

TARGET      = null-reference

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

