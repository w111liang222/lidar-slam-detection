#include "zcm/url.h"
#include "zcm/util/debug.h"

#include <cassert>
#include <string>
#include <vector>
#include <tuple>
#include <sstream>
using namespace std;

// TODO: this probably belongs more in a string util like file
static vector<string> split(const string& str, char delimiter)
{
    vector<string> v;
    std::stringstream ss {str};
    string tok;

    while(getline(ss, tok, delimiter))
        v.push_back(std::move(tok));

    auto len = str.size();
    if (len > 0 && str[len-1] == delimiter)
        v.push_back("");

    return v;
}

struct Opt
{
    string key;
    string val;

    Opt(const string& s)
    {
        // need to parse out the key and val
        size_t sep = s.find("=");
        if (sep == string::npos) {
            key = s;
            return;
        }

        // Found an '='
        key = string(s.c_str(), sep);
        val = string(s.c_str()+(sep+1), s.size()-(sep+1));
    }
};

struct zcm_url
{
    string url;

    string protocol;
    string address;
    vector<Opt> opts;

    bool zoptsPopulated = false;
    zcm_url_opts_t zopts;

    zcm_url(const char *url) : url(url) { parse(); }
    ~zcm_url() {}

    void parse()
    {
        string rest;
        size_t sep;

        sep = url.find("://");
        if (sep == string::npos) {
            protocol = url;
            return;
        }

        protocol = string(url.c_str(), sep);
        rest = string(url.c_str()+(sep+3), url.size()-(sep+3));

        sep = rest.find("?");
        if (sep == string::npos) {
            address = std::move(rest);
            return;
        }

        address = string(rest.c_str(), sep);
        rest = string(rest.c_str()+(sep+1), rest.size()-(sep+1));
        for (auto& optstr : split(rest, '&'))
            opts.emplace_back(optstr);
    }

    zcm_url_opts_t *getZopts()
    {
        if (zoptsPopulated)
            return &zopts;

        if (opts.size() > ZCM_OPTS_MAX)
            ZCM_DEBUG("WRN: dropping some url options maximum is %d", ZCM_OPTS_MAX);

        zopts.numopts = std::min(opts.size(), (size_t)ZCM_OPTS_MAX);;
        for (size_t i = 0; i < zopts.numopts; i++) {
            auto& opt = opts[i];
            zopts.name[i] = opt.key.c_str();
            zopts.value[i] = opt.val.c_str();
        }

        zoptsPopulated = true;
        return &zopts;
    }
};

extern "C" {

zcm_url_t *zcm_url_create(const char *url)
{ return new zcm_url(url); }
void zcm_url_destroy(zcm_url_t *u)
{ delete u; }

const char *zcm_url_protocol(zcm_url_t *u)
{ return u->protocol.c_str(); }
const char *zcm_url_address(zcm_url_t *u)
{ return u->address.c_str(); }
zcm_url_opts_t *zcm_url_opts(zcm_url_t *u)
{ return u->getZopts(); }

}
