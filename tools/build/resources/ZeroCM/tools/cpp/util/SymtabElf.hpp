#pragma once

#include <string>
#include <libelf.h>
#include <gelf.h>

struct SymtabElf
{
    SymtabElf(const std::string& libname);
    ~SymtabElf();

    bool good() { return elf != nullptr; }
    bool getNext(std::string& s);

  private:
    Elf         *elf;
    Elf_Scn     *scn = NULL;
    GElf_Shdr   shdr;
    Elf_Data    *data;
    int         fd, ii, count;
};
