APP_SOURCE      := app.c
APP_OBJ         := $(APP_SOURCE:%.c=%.o)
APP             := $(APP_SOURCE:%.c=%)
LIB_SOURCE      := capture_add.c
LIB_OBJ         := $(LIB_SOURCE:%.c=%.o)
LIB             := libdmtcp_$(LIB_SOURCE:%.c=%).so

TARGETS         := $(APP) $(LIB)

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

$(APP): $(APP_OBJ)
	$(CC) $(APP_OBJ) $(LDFLAGS) -o $(APP)

$(LIB): $(LIB_OBJ)
	$(CC) -shared -fPIC -o $(LIB) $(LIB_OBJ)

$(LIB_OBJ): $(LIB_SOURCE)
	$(CC) -c -fPIC -I$(DMTCP_INCLUDE) -o $(LIB_OBJ) $(LIB_SOURCE)

$(APP_OBJ): $(APP_SOURCE)
	$(CC) -c -fPIC -rdynamic -o $(APP_OBJ) $(APP_SOURCE)

#${DMTCP_ROOT}/test/dmtcp1: ${DMTCP_ROOT}/test
#	cd ${DMTCP_ROOT}/test; make dmtcp1
#check: ${LIBNAME}.so ${DMTCP_ROOT}/test/dmtcp1
#	# Kill an old coordinator on this port if present, just in case.
#	@ ${DMTCP_ROOT}/bin/dmtcp_command --quit --quiet \
#	  --coord-port ${DEMO_PORT} 2>/dev/null || true
#	${DMTCP_ROOT}/bin/dmtcp_launch --coord-port ${DEMO_PORT} --interval 5 \
#	  --with-plugin $$PWD/${LIBNAME}.so ${DMTCP_ROOT}/test/dmtcp1

