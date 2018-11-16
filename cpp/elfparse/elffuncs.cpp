
#include <elf.h>

#include <iostream>
#include <string>
#include <stdexcept>

#include <cstdio>
#include <cstring>

#define DEFAULT_CONSTR(name) \
  name() = default; \
  name(const name &other) = default; \
  name(name &&other) = default; \
  name& operator =(const name &other) = default; \
  name& operator =(name &&other) = default;

#define EXACT_COPYABLE(name, from) \
  name(const from &other) { *this = other; } \
  name& operator =(const from &other) { \
    return *this = static_cast<const name&>(other); \
  }

struct ElfHdr : public Elf64_Ehdr {
  DEFAULT_CONSTR(ElfHdr)
  EXACT_COPYABLE(ElfHdr, Elf64_Ehdr)

  ElfHdr(const Elf32_Ehdr &other) { *this = other; }
  ElfHdr& operator =(const Elf32_Ehdr &other) {
    strncpy(reinterpret_cast<char*>(this->e_ident),
            reinterpret_cast<const char*>(other.e_ident),
            EI_NIDENT);
    this->e_type     = other.e_type;
    this->e_machine  = other.e_machine;
    this->e_version  = other.e_version;
    this->e_entry    = other.e_entry;
    this->e_phoff    = other.e_phoff;
    this->e_shoff    = other.e_shoff;
    this->e_flags    = other.e_flags;
    this->e_ehsize   = other.e_ehsize;
    this->e_phentsize= other.e_phentsize;
    this->e_phnum    = other.e_phnum;
    this->e_shentsize= other.e_shentsize;
    this->e_shnum    = other.e_shnum;
    this->e_shstrndx = other.e_shstrndx;
    return *this;
  }

  bool is_32bit() const { return e_ident[EI_CLASS] == ELFCLASS32; }
  bool is_64bit() const { return e_ident[EI_CLASS] == ELFCLASS64; }
  bool is_little_endian() const { return e_ident[EI_DATA] == ELFDATA2LSB; }
  bool is_big_endian() const { return e_ident[EI_DATA] == ELFDATA2MSB; }

  std::string os_abi() const {
    switch (e_ident[EI_OSABI]) {
      case ELFOSABI_SYSV:       return "UNIX System V";
      case ELFOSABI_HPUX:       return "HP-UX";
      case ELFOSABI_NETBSD:     return "NetBSD";
      case ELFOSABI_GNU:        return "GNU";
      case ELFOSABI_SOLARIS:    return "Sun Solaris";
      case ELFOSABI_AIX:        return "IBM AIX";
      case ELFOSABI_IRIX:       return "SGI Irix";
      case ELFOSABI_FREEBSD:    return "FreeBSD";
      case ELFOSABI_TRU64:      return "Compaq TRU64 UNIX";
      case ELFOSABI_MODESTO:    return "Novell Modesto";
      case ELFOSABI_OPENBSD:    return "OpenBSD";
      case ELFOSABI_ARM_AEABI:  return "ARM EABI";
      case ELFOSABI_ARM:        return "ARM";
      case ELFOSABI_STANDALONE: return "Standalone (embedded) application";
      default:
        throw std::runtime_error("Internal error: unrecognized EI_OSABI");
    }
  }
};

struct SecHdr : public Elf64_Shdr {
  DEFAULT_CONSTR(SecHdr)
  EXACT_COPYABLE(SecHdr, Elf64_Shdr)

  SecHdr(const Elf32_Shdr &other) { *this = other; }
  SecHdr& operator =(const Elf32_Shdr &other) {
    this->sh_name      = other.sh_name;
    this->sh_type      = other.sh_type;
    this->sh_flags     = other.sh_flags;
    this->sh_addr      = other.sh_addr;
    this->sh_offset    = other.sh_offset;
    this->sh_size      = other.sh_size;
    this->sh_link      = other.sh_link;
    this->sh_info      = other.sh_info;
    this->sh_addralign = other.sh_addralign;
    this->sh_entsize   = other.sh_entsize;
    return *this;
  }
};

struct CompressionHdr : public Elf64_Chdr {
  DEFAULT_CONSTR(CompressionHdr)
  EXACT_COPYABLE(CompressionHdr, Elf64_Chdr)

  CompressionHdr(const Elf32_Chdr &other) { *this = other; }
  CompressionHdr& operator =(const Elf32_Chdr &other) {
    this->ch_type      = other.ch_type;
    this->ch_size      = other.ch_size;
    this->ch_addralign = other.ch_addralign;
    return *this;
  }
};

struct SymEntry : public Elf64_Sym {
  DEFAULT_CONSTR(SymEntry)
  EXACT_COPYABLE(SymEntry, Elf64_Sym)

  SymEntry(const Elf32_Sym &other) { *this = other; }
  SymEntry& operator =(const Elf32_Sym &other) {
    this->st_name  = other.st_name;
    this->st_info  = other.st_info;
    this->st_other = other.st_other;
    this->st_shndx = other.st_shndx;
    this->st_value = other.st_value;
    this->st_size  = other.st_size;
    return *this;
  }
};

struct SymInfo : public Elf64_Syminfo {
  DEFAULT_CONSTR(SymInfo)
  EXACT_COPYABLE(SymInfo, Elf64_Syminfo)

  SymInfo(const Elf32_Syminfo &other) { *this = other; }
  SymInfo& operator =(const Elf32_Syminfo &other) {
    this->si_boundto = other.si_boundto;
    this->si_flags   = other.si_flags;
    return *this;
  }
};

struct RelEntry : public Elf64_Rel {
  DEFAULT_CONSTR(RelEntry)
  EXACT_COPYABLE(RelEntry, Elf64_Rel)

  RelEntry(const Elf32_Rel &other) { *this = other; }
  RelEntry& operator =(const Elf32_Rel &other) {
    this->r_offset = other.r_offset;
    this->r_info   = other.r_info;
    return *this;
  }
};

struct RelEntryAddend : public Elf64_Rela {
  DEFAULT_CONSTR(RelEntryAddend)
  EXACT_COPYABLE(RelEntryAddend, Elf64_Rela)

  RelEntryAddend(const Elf32_Rela &other) { *this = other; }
  RelEntryAddend& operator =(const Elf32_Rela &other) {
    this->r_offset = other.r_offset;
    this->r_info   = other.r_info;
    this->r_addend = other.r_addend;
    return *this;
  }
};

struct ProgHdr : public Elf64_Phdr {
  DEFAULT_CONSTR(ProgHdr)
  EXACT_COPYABLE(ProgHdr, Elf64_Phdr)

  ProgHdr(const Elf32_Phdr &other) { *this = other; }
  ProgHdr& operator =(const Elf32_Phdr &other) {
    this->p_type   = other.p_type;
    this->p_offset = other.p_offset;
    this->p_vaddr  = other.p_vaddr;
    this->p_paddr  = other.p_paddr;
    this->p_filesz = other.p_filesz;
    this->p_memsz  = other.p_memsz;
    this->p_flags  = other.p_flags;
    this->p_align  = other.p_align;
    return *this;
  }
};

struct DynEntry : public Elf64_Dyn {
  DEFAULT_CONSTR(DynEntry)
  EXACT_COPYABLE(DynEntry, Elf64_Dyn)

  DynEntry(const Elf32_Dyn &other) { *this = other; }
  DynEntry& operator =(const Elf32_Dyn &other) {
    this->d_tag       = other.d_tag;
    this->d_un.d_val  = other.d_un.d_val;
    return *this;
  }
};

struct VersionDef : public Elf64_Verdef {
  DEFAULT_CONSTR(VersionDef)
  EXACT_COPYABLE(VersionDef, Elf64_Verdef)

  VersionDef(const Elf32_Verdef &other) { *this = other; }
  VersionDef& operator =(const Elf32_Verdef &other) {
    this->vd_version = other.vd_version;
    this->vd_flags   = other.vd_flags;
    this->vd_ndx     = other.vd_ndx;
    this->vd_cnt     = other.vd_cnt;
    this->vd_hash    = other.vd_hash;
    this->vd_aux     = other.vd_aux;
    this->vd_next    = other.vd_next;
    return *this;
  }
};

struct VersionAuxInfo : public Elf64_Verdaux {
  DEFAULT_CONSTR(VersionAuxInfo)
  EXACT_COPYABLE(VersionAuxInfo, Elf64_Verdaux)

  VersionAuxInfo(const Elf32_Verdaux &other) { *this = other; }
  VersionAuxInfo& operator =(const Elf32_Verdaux &other) {
    this->vda_name = other.vda_name;
    this->vda_next = other.vda_next;
    return *this;
  }
};

struct VersionDep : public Elf64_Verneed {
  DEFAULT_CONSTR(VersionDep)
  EXACT_COPYABLE(VersionDep, Elf64_Verneed)

  VersionDep(const Elf32_Verneed &other) { *this = other; }
  VersionDep& operator =(const Elf32_Verneed &other) {
    this->vn_version = other.vn_version;
    this->vn_cnt     = other.vn_cnt;
    this->vn_file    = other.vn_file;
    this->vn_aux     = other.vn_aux;
    this->vn_next    = other.vn_next;
    return *this;
  }
};

struct AuxVersionInfo : public Elf64_Vernaux {
  DEFAULT_CONSTR(AuxVersionInfo)
  EXACT_COPYABLE(AuxVersionInfo, Elf64_Vernaux)

  AuxVersionInfo(const Elf32_Vernaux &other) { *this = other; }
  AuxVersionInfo& operator =(const Elf32_Vernaux &other) {
    this->vna_hash  = other.vna_hash;
    this->vna_flags = other.vna_flags;
    this->vna_other = other.vna_other;
    this->vna_name  = other.vna_name;
    this->vna_next  = other.vna_next;
    return *this;
  }
};

struct NotesHdr : public Elf64_Nhdr {
  DEFAULT_CONSTR(NotesHdr)
  EXACT_COPYABLE(NotesHdr, Elf64_Nhdr)

  NotesHdr(const Elf32_Nhdr &other) { *this = other; }
  NotesHdr& operator =(const Elf32_Nhdr &other) {
    this->n_namesz = other.n_namesz;
    this->n_descsz = other.n_descsz;
    this->n_type   = other.n_type;
    return *this;
  }
};

struct MoveRecord : public Elf64_Move {
  DEFAULT_CONSTR(MoveRecord)
  EXACT_COPYABLE(MoveRecord, Elf64_Move)

  MoveRecord(const Elf32_Move &other) { *this = other; }
  MoveRecord& operator =(const Elf32_Move &other) {
    this->m_value   = other.m_value;
    this->m_info    = other.m_info;
    this->m_poffset = other.m_poffset;
    this->m_repeat  = other.m_repeat;
    this->m_stride  = other.m_stride;
    return *this;
  }
};

struct FileWrap {
public:
  FILE* _f;
  FileWrap(const char *filename, const char* mode)
    : _f(fopen(filename, mode))
  {
    if (_f == nullptr) {
      throw std::runtime_error(std::string("Could not open file ") + filename);
    }
  }
  ~FileWrap() { fclose(_f); }
};

class Elf {
public:
  Elf(const std::string filename)
    : _f(nullptr)
    , _fw(filename.c_str(), "r")
    , _filename(filename)
    , _is_32bit(false)
    , _is_lsb(false)
  {
    _f = _fw._f;
    parse_magic();
  }

  std::string filename() const { return _filename; }
  const unsigned char* magic() const {
    if (is_32bit()) {
      return _hdr32.e_ident;
    }
    return _hdr64.e_ident;
  }
  bool is_32bit() const { return _is_32bit; }
  bool is_64bit() const { return !_is_32bit; }
  bool is_little_endian() const { return _is_lsb; }
  bool is_big_endian() const { return !_is_lsb; }
  std::string os_abi() const { return _os_abi; }
  int abi_version() const { return _abi_version; }

  void printfuncs();

private:
  void parse_magic() {
    auto read_count = fread(&_hdr32, sizeof(Elf32_Ehdr), 1, _f);
    if (read_count < 1) {
      throw std::runtime_error("Failed to read header");
    }

    // Make sure it is an ELF file
    if (_hdr32.e_ident[0] != 0x7f ||
        _hdr32.e_ident[1] != 'E' ||
        _hdr32.e_ident[2] != 'L' ||
        _hdr32.e_ident[3] != 'F')
    {
      throw std::invalid_argument("Not an ELF file");
    }

    // Determine if it is 32-bit or 64-bit
    if (_hdr32.e_ident[EI_CLASS] == ELFCLASS32) {
      _is_32bit = true;
    } else if (_hdr32.e_ident[EI_CLASS] == ELFCLASS64) {
      _is_32bit = false;
      _hdr32 = {};
      rewind(_f);
      read_count = fread(&_hdr64, sizeof(Elf64_Ehdr), 1, _f);
      if (read_count < 1) {
        throw std::runtime_error("Failed to read header (2nd time)");
      }
    } else {
      throw std::runtime_error("Internal error: unrecognized EI_CLASS");
    }

    // Check for little endian or big endian
    if (magic()[EI_DATA] == ELFDATA2LSB) {
      _is_lsb = true;
    } else if (magic()[EI_DATA] == ELFDATA2MSB) {
      _is_lsb = false;
    }

    // Check for OS ABI
    switch (magic()[EI_OSABI]) {
      case ELFOSABI_SYSV:       _os_abi = "UNIX System V"; break;
      case ELFOSABI_HPUX:       _os_abi = "HP-UX"; break;
      case ELFOSABI_NETBSD:     _os_abi = "NetBSD"; break;
      case ELFOSABI_GNU:        _os_abi = "GNU"; break;
      case ELFOSABI_SOLARIS:    _os_abi = "Sun Solaris"; break;
      case ELFOSABI_AIX:        _os_abi = "IBM AIX"; break;
      case ELFOSABI_IRIX:       _os_abi = "SGI Irix"; break;
      case ELFOSABI_FREEBSD:    _os_abi = "FreeBSD"; break;
      case ELFOSABI_TRU64:      _os_abi = "Compaq TRU64 UNIX"; break;
      case ELFOSABI_MODESTO:    _os_abi = "Novell Modesto"; break;
      case ELFOSABI_OPENBSD:    _os_abi = "OpenBSD"; break;
      case ELFOSABI_ARM_AEABI:  _os_abi = "ARM EABI"; break;
      case ELFOSABI_ARM:        _os_abi = "ARM"; break;
      case ELFOSABI_STANDALONE: _os_abi = "Standalone (embedded) application";
                                break;
      default:
        throw std::runtime_error("Internal error: unrecognized EI_OSABI");
    }

    _abi_version = static_cast<int>(magic()[EI_ABIVERSION]);
  }

private:
  FILE* _f;           // the file for reading
  FileWrap _fw;       // will automatically close the file
  const std::string _filename;
  bool _is_32bit;     // is 32-bit
  bool _is_lsb;       // is little endian
  int _abi_version;   // abi version
  std::string _os_abi;

  Elf32_Ehdr _hdr32;
  Elf64_Ehdr _hdr64;
};

void Elf::printfuncs() {}

int main(int argCount, char *argList[]) {
  if (argCount != 2) {
    printf("Usage: %s <elf-file>\n", argList[0]);
    return 1;
  }
  try {
  Elf e(argList[1]);
  e.printfuncs();
  } catch (const std::runtime_error &ex) {
    std::cerr << "Runtime Error: " << ex.what() << std::endl;
    return 1;
  } catch (const std::invalid_argument &ex) {
    std::cerr << "Invalid Argument: " << ex.what() << std::endl;
    return 1;
  } catch (const std::exception &ex) {
    std::cerr << "Other Error: " << ex.what() << std::endl;
    return 1;
  }
  return 0;
}
