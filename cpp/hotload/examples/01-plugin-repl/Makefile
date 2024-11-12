BUILDDIR       := build
APP            := $(BUILDDIR)/plugin-repl
PLUG           := $(BUILDDIR)/libplugin.so

INTERFACE_HDR  := app/PluginFunctions.hpp
APP_SRC        := app/main.cpp
APP_SRC        += app/PluginWrapper.cpp
APP_HDRS       := $(INTERFACE_HDR)
APP_HDRS       += app/PluginWrapper.hpp
PLUG_SRC       := plugin/plugin.cpp
PLUG_HDRS      := $(INTERFACE_HDR)


# Note: GCC compiles shared libraries with "STP_GNU_UNIQUE" which ensures that
# definitions of template static data members and static local variables in
# inline functions are unique even if you specify "RTLD_LOCAL".  This
# functionality ruins the ability of "dlclose()" to actually unload the
# library.  You can use "-fno-gnu-unique" to disable this "feature".
#
#   See https://stackoverflow.com/questions/50402314/static-link-stdc-without-stb-gnu-unique-cause-memory-leak-when-dlclose

WARNFLAGS      := -Wall -Wextra -Wpedantic -Werror -Wconversion
APP_INCLUDES   := # -isystem /usr/include
APP_LINKS      := -lreadline -lfmt -lboost_filesystem
PLUG_INCLUDES  := -Iapp
PLUG_LINKS     := -fno-gnu-unique -lfmt
CXXFLAGS        = -std=c++20 -O3 -g $(WARNFLAGS)
APP_CXXFLAGS    = $(APP_INCLUDES) $(APP_LINKS) $(CXXFLAGS)
PLUG_CXXFLAGS   = $(PLUG_INCLUDES) $(PLUG_LINKS) $(CXXFLAGS)


.PHONY: default all plug app clean run
default: all
all: plug app
clean:
	$(RM) -r $(BUILDDIR)
plug: $(PLUG)
app: $(APP)

run: $(APP) $(PLUG)
	@echo "Run $(APP)"
	@./$(APP)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)

$(APP): $(APP_SRC) $(APP_HDRS) | $(BUILDDIR)
	@echo "CXX $(APP_SRC) -> $(APP)"
	@$(CXX) $(APP_CXXFLAGS) -o $(APP) $(APP_SRC)

$(PLUG): $(PLUG_SRC) $(PLUG_HDRS) | $(BUILDDIR)
	@echo "CXX $(PLUG_SRC) -> $(PLUG)"
	@$(CXX) $(PLUG_CXXFLAGS) -shared -o $(PLUG) $(PLUG_SRC)
