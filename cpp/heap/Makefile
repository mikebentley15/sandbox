CXX ?= c++
CXXFLAGS += -std=c++20 -Wall -Wextra -Wpedantic -Werror
TARGET = app
HEADER = heapq.h MinHeap.h
SOURCE = main.cpp

.PHONY: default clean
default: $(TARGET)
clean:
	rm -f $(TARGET)

$(TARGET): $(SOURCE) $(HEADER)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE)

