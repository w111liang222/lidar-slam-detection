#pragma once
#include <unistd.h>
#include <libgen.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "StringUtil.hpp"

namespace FileUtil
{
    static inline bool exists(const string& path)
    {
        struct stat st;
        return 0 == stat(path.c_str(), &st);
    }

    static inline bool dirExists(const string& path)
    {
        struct stat st;
        int ret = stat(path.c_str(), &st);
        if (ret != 0)
            return false;
        return S_ISDIR(st.st_mode);
    }

    static inline int remove(const string& path)
    {
        return unlink(path.c_str());
    }

    static inline int rename(const string& old, const string& new_)
    {
        return ::rename(old.c_str(), new_.c_str());
    }

    static inline void mkdirWithParents(const string& path, mode_t mode)
    {
        for (size_t i = 0; i < path.size(); i++) {
            if (path[i]=='/') {
                char *dirpath = (char *) malloc(i+1);
                strncpy(dirpath, path.c_str(), i);
                dirpath[i] = '\0';

                mkdir(dirpath, mode);
                free(dirpath);

                i++; // skip the '/'
            }
        }
        mkdir(path.c_str(), mode);
    }

    static inline string dirname(const string& path)
    {
        // TODO: Do this without a malloc
        char *pathCopy = strdup(path.c_str());
        string ret = ::dirname(pathCopy);
        free(pathCopy);
        return ret;
    }

    static void makeDirsForFile(const string& path)
    {
        mkdirWithParents(dirname(path), 0755);
    }
}
