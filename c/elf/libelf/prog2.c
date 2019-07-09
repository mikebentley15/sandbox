// Print the ELF Executable Header from an ELF object.

#include <err.h>
#include <fcntl.h>
#include <gelf.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <bsd/vis.h>

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

  if (elf_kind(e) != ELF_K_ELF) {
    errx(EXIT_FAILURE, "'%s' is not an ELF object.", argList[1]);
  }

  GElf_Ehdr ehdr;
  if (gelf_getehdr(e, &ehdr) == NULL) {
    errx(EXIT_FAILURE, "getehdr() failed: %s.", elf_errmsg(-1));
  }

  int i = gelf_getclass(e);
  if (i == ELFCLASSNONE) {
    errx(EXIT_FAILURE, "getclass() failed: %s.", elf_errmsg(-1));
  }

  printf("%s: %d-bit ELF object\n", argList[1], i == ELFCLASS32 ? 32 : 64);

  char *id = elf_getident(e, NULL);
  if (id == NULL) {
    errx(EXIT_FAILURE, "getident() failed: %s.", elf_errmsg(-1));
  }

  printf("%3s e_ident[0..%1d] %7s", " ", EI_ABIVERSION, " ");

  char bytes[5];
  for (i = 0; i <= EI_ABIVERSION; i++) {
    vis(bytes, id[i], VIS_WHITE, 0);
    printf(" ['%s' %X]", bytes, id[i]);
  }

  printf("\n");

#define PRINT_FMT "    %-20s 0x%jx\n"
#define PRINT_FIELD(N) printf(PRINT_FMT, #N, (uintmax_t) ehdr.N)

  PRINT_FIELD(e_type);
  PRINT_FIELD(e_machine);
  PRINT_FIELD(e_version);
  PRINT_FIELD(e_entry);
  PRINT_FIELD(e_phoff);
  PRINT_FIELD(e_shoff);
  PRINT_FIELD(e_flags);
  PRINT_FIELD(e_ehsize);
  PRINT_FIELD(e_phentsize);
  PRINT_FIELD(e_shentsize);

  size_t n;
  if (elf_getshdrnum(e, &n) != 0) {
    errx(EXIT_FAILURE, "getshdrnum() failed: %s.", elf_errmsg(-1));
  }
  printf(PRINT_FMT, "(shnum)", (uintmax_t) n);

  if (elf_getshdrstrndx(e, &n) != 0) {
    errx(EXIT_FAILURE, "getshdrstrndx() failed: %s.", elf_errmsg(-1));
  }
  printf(PRINT_FMT, "(shstrndx)", (uintmax_t) n);

  if (elf_getphdrnum(e, &n) != 0) {
    errx(EXIT_FAILURE, "getphdrnum() failed: %s.", elf_errmsg(-1));
  }
  printf(PRINT_FMT, "(phnum)", (uintmax_t) n);

  elf_end(e);
  close(fd);
  exit(EXIT_SUCCESS);

  return 0;
}
