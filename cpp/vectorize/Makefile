#CXX       := g++
#CXXFLAGS  := -O3
#CXXFLAGS  += -std=c++11
#CXXFLAGS  += -ffast-math
#CXXFLAGS  += -fopt-info-missed
#LDFLAGS   := -lm

CXX       := clang++
CXXFLAGS  := -O3
CXXFLAGS  += -std=c++11
CXXFLAGS  += -ffast-math
CXXFLAGS  += -Rpass=loop-vectorize
CXXFLAGS  += -Rpass-analysis=loop-vectorize

SOURCES   := vectorize.cpp
TARGET    := vectorize

.PHONY: all clean
all: $(TARGET)

clean:
	rm -f $(TARGET)

$(TARGET): $(SOURCES)
	$(CXX) $(CXXFLAGS) $(SOURCES) $(LDFLAGS) -o $(TARGET)

