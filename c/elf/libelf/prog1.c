// Getting started with libelf.

#include <err.h>
#include <fcntl.h>
#include <libelf.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argCount, char *argList[]) {
  if (argCount != 2) {
    errx(EXIT_FAILURE, "usage: %s file-name", argList[0]);
  }

  if (elf_version(EV_CURRENT) == EV_NONE) {
    errx(EXIT_FAILURE, "ELF library initialization failed: %s", elf_errmsg(-1));
  }

  int fd = open(argList[1], O_RDONLY, 0);
  if (fd < 0) {
    err(EXIT_FAILURE, "open '%s' failed", argList[1]);
  }

  Elf *e = elf_begin(fd, ELF_C_READ, NULL);
  if (e == NULL) {
    errx(EXIT_FAILURE, "elf_begin() failed: %s.", elf_errmsg(-1));
  }

  Elf_Kind ek = elf_kind(e);

  char *k;
  switch(ek) {
    case ELF_K_AR:
      k = "ar(1) archive";
      break;
    case ELF_K_ELF:
      k = "elf object";
      break;
    case ELF_K_NONE:
      k = "data";
      break;
    default:
      k = "unrecognized";
      break;
  }

  printf("%s: %s\n", argList[1], k);

  elf_end(e);
  close(fd);

  return EXIT_SUCCESS;
}
