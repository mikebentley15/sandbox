SOURCES   := example.cpp
TARGETS   := $(SOURCES:%.cpp=%)
CXXFLAGS  := -O0 --coverage -g
GCDA      := $(TARGETS:%=%.gcda)
GCNO      := $(TARGETS:%=%.gcno)
GCOV      := $(SOURCES:%=%.gcov)
GCOV_JSON := $(TARGETS:%=%.gcda.gcov.json.gz)
LCOV      := $(TARGETS:%=%.info)

.PHONY: default all clean gcov gcov_json lcov html
default: all
gcov: $(GCOV)
gcov_json: $(GCOV_JSON)
lcov: $(LCOV)
html: coverage/index.html
all: $(TARGETS) gcov gcov_json lcov html
clean:
	$(RM) $(TARGETS)
	$(RM) $(GCDA)
	$(RM) $(GCNO)
	$(RM) $(GCOV)
	$(RM) $(GCOV_JSON)
	$(RM) $(LCOV)
	$(RM) -r coverage

%: %.cpp
	$(CXX) $(CXXFLAGS) $< -o $@

%.gcda: %
	./$<

%.cpp.gcov: %.gcda
	gcov $<

%.gcda.gcov.json.gz: %.gcda
	gcov $< --json

%.info: $(GCOV)
	lcov --rc lcov_branch_coverage=1 --capture --directory . --output-file $@

coverage/index.html: $(LCOV)
	genhtml -o coverage/ $(LCOV)
	
