#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <elf.h>
#include <string.h>

#define AT_SEC(ehdr, shdr) ((void*)(ehdr) + (shdr)->sh_offset)

static Elf64_Shdr* section_by_name(Elf64_Ehdr* ehdr, char* name);
static Elf64_Shdr* section_by_idx(Elf64_Ehdr* ehdr, int idx);

int main(int argCount, char *argList[]) {
  if (argCount < 2) {
    fprintf(stderr, "Usage: %s elffile\n", argList[0]);
    return 1;
  }

  /* Open the file and get its size: */
  int fd = open(argList[1], O_RDONLY);
  size_t len = lseek(fd, 0, SEEK_END);

  /* Map the whole file into memory: */
  void *p = mmap(NULL, len, PROT_READ, MAP_PRIVATE, fd, 0);

  Elf64_Ehdr *ehdr = (Elf64_Ehdr *)p;

  if ((ehdr->e_ident[0] != 0x7F) ||
      (ehdr->e_ident[1] != 'E') ||
      (ehdr->e_ident[2] != 'L') ||
      (ehdr->e_ident[3] != 'F'))
  {
    printf("not an ELF file!\n");
    return 1;
  }

  printf("Type:              ");
  switch (ehdr->e_type) {
    case ET_REL:  printf("object\n"); break;
    case ET_EXEC: printf("executable\n"); break;
    case ET_DYN:  printf("shared library\n"); break;
  }

  printf("\n");

  // Print out the section header names
  {
    Elf64_Shdr *shdrs = (void*)ehdr + ehdr->e_shoff;
    char *strs = (void*)ehdr + shdrs[ehdr->e_shstrndx].sh_offset;

    printf("Section names:\n");
    int i;
    for (i = 0; i < ehdr->e_shnum; i++) {
      printf("  %s\n", strs + shdrs[i].sh_name);
    }
  }

  printf("\n");

  {
    Elf64_Shdr *dynsym_shdr = section_by_name(ehdr, ".dynsym");
    Elf64_Shdr *dynstr_shdr = section_by_name(ehdr, ".dynstr");
    if (dynsym_shdr != NULL || dynstr_shdr != NULL) {
      Elf64_Sym *syms = AT_SEC(ehdr, dynsym_shdr);
      char *strs = AT_SEC(ehdr, dynstr_shdr);

      printf("Dynamic Symbols:\n");
      int count = dynsym_shdr->sh_size / sizeof(Elf64_Sym);
      int i;
      for (i = 0; i < count; i++) {
        printf("  %s\n", strs + syms[i].st_name);
      }
    } else {
      printf("Dynamic Symbols:   None\n");
    }
  }

  return 0;
}

Elf64_Shdr* section_by_name(Elf64_Ehdr* ehdr, char* name) {
  Elf64_Shdr *shdrs = (void*)ehdr + ehdr->e_shoff;
  char *strs = (void*)ehdr + shdrs[ehdr->e_shstrndx].sh_offset;
  int i;
  for (i = 0; i < ehdr->e_shnum; i++) {
    if (0 == strcmp(name, strs + shdrs[i].sh_name)) {
      return shdrs + i;
    }
  }
  return NULL;
}

Elf64_Shdr* section_by_idx(Elf64_Ehdr* ehdr, int idx) {
  if (idx < 0 || idx >= ehdr->e_shnum) {
    return NULL;
  }
  Elf64_Shdr *shdrs = (void*)ehdr + ehdr->e_shoff;
  return shdrs + idx;
}
