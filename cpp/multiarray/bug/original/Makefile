SOURCE       := H1.cpp
PREPROCESSED := preprocessed.cpp
MINIMAL      := minimal.cpp
TEST         := test.sh

CREDUCE      := creduce
CP           := cp
RM           := rm -f
CPP          := g++ -E

.PHONY: all clean
all: $(MINIMAL)
clean:
	$(RM) $(MINIMAL)
	$(RM) $(PREPROCESSED)
	$(RM) tmp*

$(MINIMAL): $(PREPROCESSED) $(TEST)
	@$(CP) $(PREPROCESSED) $(MINIMAL)
	$(CREDUCE) ./$(TEST) $(MINIMAL)
	@$(RM) $(MINIMAL).orig

$(PREPROCESSED): $(SOURCE)
	$(CPP) $(SOURCE) -o $(PREPROCESSED)
