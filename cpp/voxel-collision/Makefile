CXX           ?= g++
CXXFLAGS      ?= -O3

TARGET        := voxel-collision-example
SOURCE        := main.cpp
HEADERS       := VoxelObject.h

.PHONY: all clean

all: $(TARGET)
clean:
	rm -f $(TARGET)

$(TARGET): $(SOURCE) $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE)
