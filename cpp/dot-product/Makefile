CXX         = g++
CFLAGS      = -Wall -Wextra
CFLAGS_1   := $(CFLAGS) -O2
CFLAGS_2   := $(CFLAGS) -O3 -funsafe-math-optimizations -msse2

SOURCE     := dot-product.cpp
SOURCE     += main.cpp
OBJ_1      := $(SOURCE:%.cpp=%-1.o)
OBJ_2      := $(SOURCE:%.cpp=%-2.o)
OBJ        := $(OBJ_1) $(OBJ_2)
TARGET_1   := dot-product-1
TARGET_2   := dot-product-2
TARGETS    := $(TARGET_1) $(TARGET_2)

.PHONY: all clean run
all: $(TARGETS)

clean:
	rm -f $(TARGETS)
	rm -f $(OBJ)

run: $(TARGETS)
	./$(TARGET_1)
	./$(TARGET_2)

$(TARGET_1): $(OBJ_1) Makefile
	$(CXX) $(CFLAGS_1) $(OBJ_1) -o $(TARGET_1)

$(TARGET_2): $(OBJ_2) Makefile
	$(CXX) $(CFLAGS_2) $(OBJ_2) -o $(TARGET_2)

%-1.o: %.cpp Makefile
	$(CXX) $(CFLAGS_1) -c -o $@ $<

%-2.o: %.cpp Makefile
	$(CXX) $(CFLAGS_2) -c -o $@ $<
