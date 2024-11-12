SOURCE    := main.cpp
HEADERS   := TreeChecker.h
CXXFLAGS  := -std=c++20 -Wall -Wextra -Wpedantic -Werror
TARGET    := app

# Be silent by default
ifndef VERBOSE
.SILENT:
endif

.PHONY: default clean run
default: run
clean:
	$(RM) $(TARGET)
run: $(TARGET)
	./$(TARGET)

$(TARGET): $(SOURCE) $(HEADERS) Makefile
	$(CXX) $(CXXFLAGS) $(SOURCE) -o $(TARGET)
