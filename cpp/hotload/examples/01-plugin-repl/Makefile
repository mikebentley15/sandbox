BUILDDIR          := build
APP               := $(BUILDDIR)/plugin-repl
TEST_PLUG         := $(BUILDDIR)/libTestPlugin.so
WORDLE_PLUG       := $(BUILDDIR)/libWordlePlugin.so

INTERFACE_HDR     := app/PluginFunctions.hpp
APP_SRC           := app/main.cpp
APP_SRC           += app/PluginWrapper.cpp
APP_HDRS          := $(INTERFACE_HDR)
APP_HDRS          += app/PluginWrapper.hpp
TEST_PLUG_SRC     := TestPlugin/TestPlugin.cpp
TEST_PLUG_HDRS    := $(INTERFACE_HDR)
WORDLE_WORD_LISTS := WordlePlugin/wordle_allowed.txt
WORDLE_WORD_LISTS += WordlePlugin/wordle_answers.txt
WORDLE_PLUG_SRC   := WordlePlugin/WordlePlugin.cpp
WORDLE_PLUG_HDRS  := $(INTERFACE_HDR)
WORDLE_PLUG_HDRS  += $(WORDLE_WORD_LISTS:WordlePlugin/%.txt=build/%_wrap.h)

# Note: GCC compiles shared libraries with "STP_GNU_UNIQUE" which ensures that
# definitions of template static data members and static local variables in
# inline functions are unique even if you specify "RTLD_LOCAL".  This
# functionality ruins the ability of "dlclose()" to actually unload the
# library.  You can use "-fno-gnu-unique" to disable this "feature".
#
#   See https://stackoverflow.com/questions/50402314/static-link-stdc-without-stb-gnu-unique-cause-memory-leak-when-dlclose

WARNFLAGS      := -Wall -Wextra -Wpedantic -Werror -Wconversion
CXXFLAGS        = -I. -std=c++20 -O3 -g $(WARNFLAGS)

APP_INCLUDES   := # -isystem /usr/include
APP_LINKS      := -lreadline -lfmt -lboost_filesystem
APP_CXXFLAGS    = $(APP_INCLUDES) $(APP_LINKS) $(CXXFLAGS)

PLUG_LINKS     := -fPIC -fno-gnu-unique

TEST_PLUG_INCLUDES  :=
TEST_PLUG_LINKS     := $(PLUG_LINKS) -lfmt
TEST_PLUG_CXXFLAGS   = $(TEST_PLUG_INCLUDES) $(TEST_PLUG_LINKS) $(CXXFLAGS)

WORDLE_PLUG_INCLUDES :=
WORDLE_PLUG_LINKS    := $(PLUG_LINKS) -lfmt
WORDLE_PLUG_CXXFLAGS  = $(WORDLE_PLUG_INCLUDES) $(WORDLE_PLUG_LINKS) $(CXXFLAGS)

.PHONY: default all plug app clean run
default: all
all: plug app
clean:
	$(RM) -r $(BUILDDIR)
plug: $(TEST_PLUG) $(WORDLE_PLUG)
app: $(APP)

run: $(APP) $(TEST_PLUG) $(WORDLE_PLUG)
	@echo "Run $(APP)"
	@./$(APP)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(APP): $(APP_SRC) $(APP_HDRS) | $(BUILDDIR)
	@echo "CXX $(APP_SRC) -> $(APP)"
	@$(CXX) $(APP_CXXFLAGS) -o $(APP) $(APP_SRC)

$(TEST_PLUG): $(TEST_PLUG_SRC) $(TEST_PLUG_HDRS) | $(BUILDDIR)
	@echo "CXX $(TEST_PLUG_SRC) -> $(TEST_PLUG)"
	@$(CXX) $(TEST_PLUG_CXXFLAGS) -shared -o $(TEST_PLUG) $(TEST_PLUG_SRC)

$(WORDLE_PLUG): $(WORDLE_PLUG_SRC) $(WORDLE_PLUG_HDRS) | $(BUILDDIR)
	@echo "CXX $(WORDLE_PLUG_SRC) -> $(WORDLE_PLUG)"
	@$(CXX) $(WORDLE_PLUG_CXXFLAGS) -shared -o $(WORDLE_PLUG) $(WORDLE_PLUG_SRC)

build/%_wrap.h: WordlePlugin/%.txt Makefile
	@echo "WRAP $< -> $@"
	@sed -e 's/^\(.*\)$$/"\1",/g' $< > $@
