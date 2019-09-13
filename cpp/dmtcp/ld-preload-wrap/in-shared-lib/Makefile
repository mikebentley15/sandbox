APP_SOURCE      := app.c
APP_OBJ         := $(APP_SOURCE:%.c=%.o)
APP             := $(APP_SOURCE:%.c=%)
LIB_SOURCE      := addlib.c
LIB_OBJ         := $(LIB_SOURCE:%.c=%.o)
LIB             := lib$(LIB_SOURCE:%.c=%).so
PLUGIN_SOURCE   := capture_add.c
PLUGIN_OBJ      := $(PLUGIN_SOURCE:%.c=%.o)
PLUGIN          := libdmtcp_$(PLUGIN_SOURCE:%.c=%).so

TARGETS         := $(APP) $(LIB) $(PLUGIN)

ifndef DMTCP_ROOT
DMTCP_ROOT      := ..
endif

DMTCP_INCLUDE   := ${DMTCP_ROOT}/include

override CFLAGS += -fPIC -I${DMTCP_INCLUDE}
override CXXFLAGS += -fPIC -I${DMTCP_INCLUDE}
LINK = ${CC}

DEMO_PORT=7781

.PHONY: default clean
default: $(TARGETS)
clean:
	$(RM) $(TARGETS)
	$(RM) $(APP_OBJ)
	$(RM) $(LIB_OBJ)
	$(RM) $(PLUGIN_OBJ)

$(APP): $(APP_OBJ) $(LIB) Makefile
	$(CC) $(APP_OBJ) $(LDFLAGS) -L. -l$(LIB:lib%.so=%) -Wl,-rpath=$(PWD) -o $(APP)

$(LIB): $(LIB_OBJ) Makefile
	$(CC) -shared -fPIC -o $(LIB) $(LIB_OBJ)

$(PLUGIN): $(PLUGIN_OBJ) Makefile
	$(CC) -shared -fPIC -o $(PLUGIN) $(PLUGIN_OBJ)

$(APP_OBJ): $(APP_SOURCE) Makefile
	$(CC) -c -fPIC -rdynamic -o $(APP_OBJ) $(APP_SOURCE)

$(LIB_OBJ): $(LIB_SOURCE) Makefile
	$(CC) -c -fPIC -o $(LIB_OBJ) $(LIB_SOURCE)

$(PLUGIN_OBJ): $(PLUGIN_SOURCE) Makefile
	$(CC) -c -fPIC -I$(DMTCP_INCLUDE) -o $(PLUGIN_OBJ) $(PLUGIN_SOURCE)

#${DMTCP_ROOT}/test/dmtcp1: ${DMTCP_ROOT}/test
#	cd ${DMTCP_ROOT}/test; make dmtcp1
#check: ${LIBNAME}.so ${DMTCP_ROOT}/test/dmtcp1
#	# Kill an old coordinator on this port if present, just in case.
#	@ ${DMTCP_ROOT}/bin/dmtcp_command --quit --quiet \
#	  --coord-port ${DEMO_PORT} 2>/dev/null || true
#	${DMTCP_ROOT}/bin/dmtcp_launch --coord-port ${DEMO_PORT} --interval 5 \
#	  --with-plugin $$PWD/${LIBNAME}.so ${DMTCP_ROOT}/test/dmtcp1

