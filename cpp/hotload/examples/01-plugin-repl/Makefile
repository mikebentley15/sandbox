APP            := plugin-repl
APP_SRC        := main.cpp
PLUG           := libplugin.so
PLUG_SRC       := plugin.cpp
WARNFLAGS      := -Wall -Wextra -Wpedantic -Werror -Wconversion
APP_INCLUDES   := # -isystem /usr/include
APP_LINKS      := -lreadline
PLUG_INCLUDES  :=
PLUG_LINKS     :=
CXXFLAGS        = -std=c++20 -O3 -g $(WARNFLAGS)
APP_CXXFLAGS    = $(APP_INCLUDES) $(APP_LINKS) $(CXXFLAGS)
PLUG_CXXFLAGS   = $(PLUG_INCLUDES) $(PLUG_LINKS) $(CXXFLAGS)

.PHONY: default all plug app clean run
default: all
all: plug app
clean:
	$(RM) $(APP) $(PLUG)
plug: $(PLUG)
app: $(APP)

run: $(APP) $(PLUG)
	@echo "Run $(APP)"
	@./$(APP)

$(APP): $(APP_SRC)
	@echo "CXX $(APP_SRC) -> $(APP)"
	@$(CXX) $(APP_CXXFLAGS) -o $(APP) $(APP_SRC)

$(PLUG): $(PLUG_SRC)
	@echo "CXX $(PLUG_SRC) -> $(PLUG)"
	@$(CXX) $(PLUG_CXXFLAGS) -shared -o $(PLUG) $(PLUG_SRC)
