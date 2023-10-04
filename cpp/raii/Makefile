SOURCES := main.cpp
HEADERS := ScopeCleaner.hpp
TARGET  := scopecleaner
CXXFLAGS := -O3 -std=c++20

.PHONY: all default clean
default: all
all: $(TARGET)
clean:
	$(RM) $(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SOURCES) -o $(TARGET)
