SOURCE   += main.cpp
HEADERS  += LockFreeQueue.h
HEADERS  += LockFreeQueue_MMMS.h
#CXXFLAGS += -O3 -Wall -Wextra -Wpedantic -Wconversion -Werror
CXXFLAGS += -g -O0 -Wall -Wextra -Wpedantic -Wconversion -Werror
TARGET   := main

.PHONY: default clean
default: $(TARGET)
clean:
	$(RM) $(TARGET)

$(TARGET): $(SOURCE) $(HEADERS) Makefile
	$(CXX) $(CXXFLAGS) $(SOURCE) -o $(TARGET)
