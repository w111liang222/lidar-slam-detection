#include "zcm/util/lockfile.h"
#include "zcm/util/debug.h"

#include <cstring>
#include <fcntl.h>
#include <pwd.h>
#include <signal.h>
#include <stdlib.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <unordered_map>

using namespace std;

#define DEFAULT_LOCK_DIR "/var/lock/zcm"
#define LOCK_PREFIX "LCK.."

static bool startsWith(const string& s, const string& pre)
{
    if (s.size() < pre.size())
        return false;
    for (size_t i = 0; i < pre.size(); i++)
        if (s[i] != pre[i])
            return false;
    return true;
}

static string makeLockfilePath(const string& name)
{
    string ret = DEFAULT_LOCK_DIR;

    const char* dir = std::getenv("ZCM_LOCK_DIR");
    if (dir) ret = dir;

    int err = mkdir(ret.c_str(), S_IRWXO | S_IRWXG | S_IRWXU);
    if (err < 0 && errno != EEXIST) {
        ZCM_DEBUG("Unable to create lockfile directory");
        return "";
    }

    if (ret == "") return ret;

    if (ret[ret.size() - 1] != '/') ret += "/";

    ret += LOCK_PREFIX;

    size_t idx = 0;
    if (startsWith(name, "/dev/"))
        idx = 5;
    for (; idx < name.size(); idx++) {
        auto c = name[idx];
        if (c == '/')
            ret += string(1, '_');
        else
            ret += string(1, c);
    }
    return ret;
}

lockfile_t* lockfile_trylock(const char *name)
{
    auto path = makeLockfilePath(name);
    int fd = open(path.c_str(), O_WRONLY | O_CREAT, 0666);
    if (fd < 0) {
        ZCM_DEBUG("Unable to open lockfile: '%s'", name);
        return nullptr;
    }

    int lkRet = lockf(fd, F_TLOCK, 0);
    if (lkRet < 0) {
        ZCM_DEBUG("Failed to lock lockfile '%s'. Error: %s", name, strerror(errno));
        close(fd);
        return nullptr;
    }

    lockfile_t *ret = new lockfile_t;
    ret->fd = fd;
    ret->name = strdup(name);
    return ret;
}

void lockfile_unlock(lockfile_t* lf)
{
    int ret = lockf(lf->fd, F_ULOCK, 0);
    if (ret < 0)
        ZCM_DEBUG("Failed to unlock lockfile '%s'. Error: %s", lf->name, strerror(errno));

    close(lf->fd);

    int err = unlink(makeLockfilePath(lf->name).c_str());
    if (err < 0)
        ZCM_DEBUG("Failed to unlink lockfile '%s'. Error: %s", lf->name, strerror(errno));

    free(lf->name);
    delete lf;
}
