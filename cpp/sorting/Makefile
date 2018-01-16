CXX        := g++ -std=c++14 -O3
SOURCES    := $(wildcard *.cpp)
OBJ        := $(SOURCES:%.cpp=%.o)
TARGET     := sort-timings
DOC_CMD    := pandoc --standalone --mathjax
DOC_IN     := $(wildcard *.md)
DOC_OUT    := $(DOC_IN:%.md=%.html)

.PHONY: all clean distclean veryclean doc
all: $(TARGET) doc
doc: $(DOC_OUT)
clean:
	rm -f $(TARGET)
	rm -f $(DOC_OUT)
	rm -f $(OBJ)

$(TARGET) : $(OBJ)
	$(CXX) -o $@ $(OBJ)

%.o : %.cpp Makefile
	$(CXX) -c -o $@ $<

%.html : %.md
	$(DOC_CMD) -o $@ $<
