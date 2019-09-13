#include <stdio.h>
#include <sys/time.h>
#include "dmtcp.h"

#include <unistd.h>

int __real_add(int a, int b);

int __wrap_add(int a, int b) {
  int original_generation = dmtcp_get_generation();
  int status = dmtcp_checkpoint();

  switch(status) {

    case DMTCP_AFTER_CHECKPOINT:
      while (dmtcp_get_generation() == original_generation) { sleep(1); }
      break;

    case DMTCP_AFTER_RESTART:
      printf("* After restart\n");
      break;

    default:
    case DMTCP_NOT_PRESENT:
      printf("ERROR: DMTCP is not running - why are you using this plugin?\n");
      break;

  }

  //int result = NEXT_FNC(add)(a, b);
  int result = __real_add(a, b);

  return result;
}

static DmtcpBarrier barriers[] = {};

DmtcpPluginDescriptor_t capture_add = {
  DMTCP_PLUGIN_API_VERSION,
  DMTCP_PACKAGE_VERSION,
  "capture_add",
  "DMTCP",
  "mikebentley15@gmail.com",
  "Capture add() plugin",
  DMTCP_DECL_BARRIERS(barriers),
  NULL
};

DMTCP_DECL_PLUGIN(capture_add);
