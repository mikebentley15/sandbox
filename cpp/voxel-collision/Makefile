CXX           ?= g++
CXXFLAGS      ?= -O3 -std=c++17 -g

TARGET        := voxel-collision-example
SOURCE        := main.cpp
HEADERS       := VoxelObject.h SparseVoxelObject.h CTVoxelObject.h

.PHONY: all clean

all: $(TARGET)
clean:
	rm -f $(TARGET)

$(TARGET): $(SOURCE) $(HEADERS) Makefile
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE)
