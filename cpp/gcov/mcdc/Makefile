SOURCE    := mcdc_demo.cpp
TARGET    := $(SOURCE:%.cpp=%)
COMP_DATA := $(TARGET:%=%.gcno)
RUN_DATA  := $(SOURCE:%=%.gcov)
RUN_DATA  += $(TARGET:%=%.gcda)
CXXFLAGS  += -std=c++11 -Wall -Wextra -Wpedantic -Werror
CXXFLAGS  += -g -O0 -fprofile-arcs -ftest-coverage
GCOV      := gcov
GCOVFLAGS += -pcb

.PHONY: default build clean report decision condition mcdc
default: build
build: $(TARGET)
clean:
	$(RM) $(TARGET)
	$(RM) $(COMP_DATA)
	$(RM) $(RUN_DATA)
	$(RM) *.gcov

$(TARGET): $(SOURCE) Makefile
	$(CXX) $(CXXFLAGS) $(SOURCE) -o $(TARGET)

report: $(TARGET)
	$(GCOV) $(GCOVFLAGS) $(SOURCE)
	cat $(SOURCE:%=%.gcov)

decision: $(TARGET)
	$(RM) $(RUN_DATA)
	./$(TARGET) 1 2 3
	$(MAKE) report

condition: $(TARGET)
	$(RM) $(RUN_DATA)
	./$(TARGET) 1 2 3
	./$(TARGET) 3 2 1
	./$(TARGET) 1 3 2
	$(MAKE) report

mcdc: $(TARGET)
	$(RM) $(RUN_DATA)
	#./$(TARGET) 1 2 3
	#./$(TARGET) 3 2 1
	#./$(TARGET) 3 1 2
	#./$(TARGET) 1 3 2
	#./$(TARGET) 2 1 3
	# error cases (not integers)
	./$(TARGET) "" b c
	./$(TARGET) words b c
	./$(TARGET) 99999999999999999999 b c
	#./$(TARGET) a b c || true
	#./$(TARGET) 1 b c || true
	#./$(TARGET) 1 1 c || true
	## error cases (out of bounds)
	#./$(TARGET) 99999999999999999999 99999999999999999999 99999999999999999999 || true
	#./$(TARGET) 1 99999999999999999999 99999999999999999999 || true
	#./$(TARGET) 1 1 99999999999999999999 || true
	$(MAKE) report
