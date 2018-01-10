LIB_CXX       = g++
LIB_CFLAGS    = -fPIC -I. -g -fno-inline
LIB_LINKFLAGS = -shared
LIB_TARGET    = libbugrepro.so
LIB_SOURCES   = library.cpp
LIB_HEADERS   = library.h

CLANG_VERSIONS += 3.9.0
CLANG_VERSIONS += 3.9.1
CLANG_VERSIONS += 4.0.0
CLANG_VERSIONS += 4.0.1
CLANG_VERSIONS += 5.0.0

APP_CXX_PREFIX  = clang++

#APP_CXX       = clang++-3.9.0
#APP_CXX       = clang++-3.9.1
#APP_CXX       = clang++-4.0.0
#APP_CXX       = clang++-4.0.1
#APP_CXX       = clang++-5.0.0
APP_CFLAGS    = -O2
APP_LINKFLAGS = -L. -lbugrepro -Wl,-rpath=.
APP_TARGET_PREFIX = bugrepro
APP_TARGETS   = $(addprefix $(APP_TARGET_PREFIX)-,$(CLANG_VERSIONS))
APP_SOURCES   = main.cpp

.PHONY: all lib app clean run
all: lib app run
lib: $(LIB_TARGET)
app: $(APP_TARGETS)
clean:
	rm -f $(LIB_TARGET)
	rm -f $(APP_TARGETS)
run: $(APP_TARGETS)
	$(foreach a,$(APP_TARGETS),./$a &&) true

$(APP_TARGETS): $(APP_SOURCES) $(LIB_HEADERS) $(LIB_TARGET) makefile
	$(APP_CXX_PREFIX)-$(@:$(APP_TARGET_PREFIX)-%=%) $(APP_CFLAGS) -o $@ $(APP_SOURCES) $(APP_LINKFLAGS)

$(LIB_TARGET): $(LIB_SOURCES) $(LIB_HEADERS) makefile
	$(LIB_CXX) $(LIB_CFLAGS) -o $(LIB_TARGET) $(LIB_SOURCES) $(LIB_LINKFLAGS)

