CXX    := nvcc
SRC    := main.cu
TARGET := hello-world

.PHONY: all clean
all: $(TARGET)
clean:
	rm -f $(TARGET)

$(TARGET): $(SRC)
	$(CXX) -o $(TARGET) $(SRC)
