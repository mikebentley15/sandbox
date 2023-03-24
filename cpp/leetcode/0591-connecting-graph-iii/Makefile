SOURCE    := main.cpp
HEADERS   := ConnectingGraph3.h UnionFind.h
CXXFLAGS  += -std=c++20 -Wall -Wextra -Wpedantic -Werror
TARGET    := app

.PHONY: default clean
default: $(TARGET)
clean:
	$(RM) $(TARGET)

$(TARGET): $(SOURCE) $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE)
