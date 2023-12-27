#include "SymtabElf.hpp"

#include <cstdio>
#include <cstdint>
#include <cassert>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "zcm/util/debug.h"

SymtabElf::SymtabElf(const std::string& libname)
{
    elf_version(EV_CURRENT);

    fd = open(libname.c_str(), O_RDONLY);
    if (!fd){
        ZCM_DEBUG("Unable to open fd");
        return;
    }

    elf = elf_begin(fd, ELF_C_READ, NULL);
    if (!elf) {
        ZCM_DEBUG("Unable to open elf");
        close(fd);
        fd = 0;
        return;
    }

    while ((scn = elf_nextscn(elf, scn)) != NULL) {
        gelf_getshdr(scn, &shdr);
        if (shdr.sh_type == SHT_SYMTAB) {
            break;
        }
    }

    if (shdr.sh_type != SHT_SYMTAB) {
        ZCM_DEBUG("Unable to find symbol table");
        elf_end(elf);
        elf = nullptr;
        close(fd);
        fd = 0;
    }

    data = elf_getdata(scn, NULL);
    count = shdr.sh_size / shdr.sh_entsize;
    ii = 0;

    ZCM_DEBUG("Number of symbols in elf: %d", count);
}

SymtabElf::~SymtabElf()
{
    elf_end(elf);
    elf = nullptr;

    close(fd);
    fd = 0;
}

// a very crude symbol extraction method
// we simply search for byte sequences that look like valid
// C language identifiers
bool SymtabElf::getNext(std::string& s)
{
    if (ii == count) return false;

    s.clear();

    GElf_Sym sym;
    gelf_getsym(data, ii, &sym);
    ii++;

    char* str = elf_strptr(elf, shdr.sh_link, sym.st_name);
    s = std::string(str);

    return true;
}
