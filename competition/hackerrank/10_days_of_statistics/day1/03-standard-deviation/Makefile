CXX        := g++ -std=c++14
SOURCES    := $(wildcard *.cpp)
TARGETS    := $(SOURCES:%.cpp=%)
DOC_CMD    := pandoc --standalone --mathjax
DOC_IN     := $(wildcard *.md)
DOC_OUT    := $(DOC_IN:%.md=%.html)

.PHONY: all clean distclean veryclean doc
all: $(TARGETS) doc
doc: $(DOC_OUT)
clean:
	rm -f $(TARGETS)
	rm -f $(DOC_OUT)

% : %.cpp
	$(CXX) -o $@ $<

%.html : %.md
	$(DOC_CMD) -o $@ $<
