CXX          := clang++
CXXFLAGS     := -g -O0 -std=c++11 -Wall -Wextra -Werror
LDFLAGS      :=
SOURCE       := cpp-hier-parser.cpp
TARGET       := $(SOURCE:%.cpp=%)
TESTIN       := $(wildcard testin-*.cpp)
TEST_TARGETS := $(TESTIN:%=test__%)

.PHONY: all clean test check $(TEST_TARGETS)
all: $(TARGET)
clean:
	rm -f $(TARGET)
	rm -f *_actual.cpp
check: test

# After testing particular inputs, test that parsing the source works
test: $(TARGET) $(TEST_TARGETS)
	./$(TARGET) $(SOURCE) $(SOURCE)_parsed_actual.cpp
	@rm -f $(SOURCE)_parsed_actual.cpp

$(TARGET): $(SOURCE) Makefile
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE) $(LDFLAGS)

.PHONY: $(TEST_TARGETS)
$(TEST_TARGETS): $(TARGET)
	./$(TARGET) $(subst test__,,$@) $(subst in,out,$(subst test__,,$@))_actual.cpp
	diff -Nuir $(subst in,out,$(subst test__,,$@))_actual.cpp $(subst in,out,$(subst test__,,$@))
	@rm -f $(subst in,out,$(subst test__,,$@))_actual.cpp
