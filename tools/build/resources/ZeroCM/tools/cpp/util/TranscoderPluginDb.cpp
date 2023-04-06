#include <cstring>
#include <algorithm>
#include <memory>
#include <cxxabi.h>
#include <typeinfo>
#include <dlfcn.h>

#include "zcm/util/Common.hpp"
#include "util/StringUtil.hpp"

#include "SymtabElf.hpp"

#include "TranscoderPluginDb.hpp"

#define DEBUG(...) do {\
    if (this->debug) printf(__VA_ARGS__);\
  } while(0)

using namespace std;

static inline std::string demangle(std::string name)
{
    int status = 42; // some arbitrary value to eliminate the compiler warning

    std::unique_ptr<char, void(*)(void*)> res {
        abi::__cxa_demangle(name.c_str(), nullptr, nullptr, &status),
        std::free
    };

    return (status==0) ? res.get() : name ;
}

static void* openlib(const string& libname)
{
    // verify the .so library
    if (!StringUtil::endswith(libname, ".so")) {
        ERROR("bad library name, expected a .so file, not '%s'\n", libname.c_str());
        return nullptr;
    }

    // attempt to open the .so
    void* lib = dlopen(libname.c_str(), RTLD_LAZY);
    if (!lib) {
        ERROR("failed to open '%s'\n", libname.c_str());
        ERROR("%s\n", dlerror());
        return nullptr;
    }

    return lib;
}

static string method = "::makeTranscoderPlugin()";

// find all zcm types by post-processing the symbols
bool TranscoderPluginDb::findPlugins(const string& libname)
{
    void* lib = openlib(libname);
    if (lib == nullptr) {
        ERROR("failed to open '%s'\n", libname.c_str());
        return false;
    }

    // read the library's symbol table
    SymtabElf stbl{libname};
    if (!stbl.good()) {
        ERROR("ERR: failed to load symbol table for ELF file\n");
        return false;
    }

    // process the symbols
    string s;
    while (stbl.getNext(s)) {
        string demangled = demangle(s);
        //DEBUG("%s\n", demangled.c_str());
        if (!StringUtil::endswith(demangled, method)) continue;

        TranscoderPluginMetadata md;
        size_t methodPos = demangled.find(method);
        assert(methodPos != string::npos && "Shouldn't be able to get here");
        md.className = demangled.substr(0, methodPos);
        md.makeTranscoderPlugin = nullptr;

        if (std::find(pluginMeta.begin(), pluginMeta.end(), md) != pluginMeta.end()) continue;

        *((void **) &md.makeTranscoderPlugin) = dlsym(lib, s.c_str());
        if (md.makeTranscoderPlugin == nullptr) {
            ERROR("ERR: failed to load %s\n", s.c_str());
            continue;
        }

        pluginMeta.push_back(md);

        DEBUG("Success loading plugin %s\n", demangled.c_str());
    }

    for (auto& meta : pluginMeta) {
        zcm::TranscoderPlugin* p = (zcm::TranscoderPlugin*) meta.makeTranscoderPlugin();
        DEBUG("Added new plugin with address %p\n", p);
        plugins.push_back(p);
        constPlugins.push_back(plugins.back());
        names.push_back(meta.className);
    }

    DEBUG("Loaded %d plugins from %s\n", (int)plugins.size(), libname.c_str());

    return true;
}

std::vector<const zcm::TranscoderPlugin*> TranscoderPluginDb::getPlugins() const
{ return constPlugins; }

std::vector<string> TranscoderPluginDb::getPluginNames() const
{ return names; }

TranscoderPluginDb::TranscoderPluginDb(const string& paths, bool debug) : debug(debug)
{
    for (auto& libname : StringUtil::split(paths, ':')) {
        DEBUG("Loading plugins from '%s'\n", libname.c_str());
        if (!findPlugins(libname)) {
            ERROR("failed to find indexer plugins in %s\n", libname.c_str());
            continue;
        }
    }
}

TranscoderPluginDb::~TranscoderPluginDb() { for (auto p : plugins) delete p; }
