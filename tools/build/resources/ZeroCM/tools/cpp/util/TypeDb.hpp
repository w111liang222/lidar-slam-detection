#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>

#include <zcm/zcm_coretypes.h>

struct TypeMetadata
{
    int64_t hash;
    std::string name;
    const zcm_type_info_t* info;
};

class TypeDb
{
  public:
    TypeDb(const std::string& paths, bool debug=false);

    const TypeMetadata* getByHash(int64_t hash) const;
    const TypeMetadata* getByName(const std::string& name) const;

    bool good() const;

  private:
    bool findTypenames(std::vector<std::string>& result, const std::string& libname);
    bool loadtypes(const std::string& libname, void* lib);

  private:
    bool debug;
    bool isGood;
    std::unordered_map<int64_t, TypeMetadata> hashToType;
    std::unordered_map<std::string, int64_t>  nameToHash;
};
