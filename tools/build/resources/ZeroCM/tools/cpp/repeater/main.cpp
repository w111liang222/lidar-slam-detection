#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <cinttypes>
#include <regex>

#include <errno.h>
#include <time.h>
#include <getopt.h>

#include "zcm/zcm.h"

#include <string>

using namespace std;

struct Args
{
    string chan          = ".*";
    string src_url       = "";
    string dest_url      = "";
    bool invert_channels = false;

    bool parse(int argc, char *argv[])
    {
        // set some defaults
        const char *optstring = "hc:is:d:";
        struct option long_opts[] = {
            { "help",                  no_argument, 0, 'h' },
            { "channel",         required_argument, 0, 'c' },
            { "invert-channels",       no_argument, 0, 'i' },
            { "src-url",         required_argument, 0, 's' },
            { "dest-url",        required_argument, 0, 'd' },
            { 0, 0, 0, 0 }
        };

        int c;
        while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0) {
            switch (c) {
                case 'c': chan            = optarg; break;
                case 's': src_url         = optarg; break;
                case 'd': dest_url        = optarg; break;
                case 'i': invert_channels = true;   break;
                case 'h': default: usage(); return false;
            };
        }

        if (src_url == "") {
            fprintf(stderr, "Please specify a source transport with the -s option\n");
            return false;
        }

        if (dest_url == "") {
            fprintf(stderr, "Please specify a destination transport with the -d option\n");
            return false;
        }

        if (src_url == dest_url) {
            fprintf(stderr, "Destination network and source transports must be unique\n");
            return false;
        }

        return true;
    }

    void usage()
    {
        fprintf(stderr, "usage: zcm-repeater [options]\n"
                "\n"
                "    ZCM repeater utility.  Subscribes to all channels on a source ZCM\n"
                "    network, and repeats all messages received on that network to\n"
                "    a destination ZCM network.\n"
                "\n"
                "Example:\n"
                "    zcm-repeater -s ipc -d udpm://224.255.76.67:7667?ttl=0\n"
                "\n"
                "Options:\n"
                "\n"
                "  -c, --channel=CHAN         Channel string to pass to zcm_subscribe.\n"
                "                             (default: \".*\")\n"
                "  -h, --help                 Shows this help text and exits\n"
                "  -s, --src-url=URL          Subscribe to messages on the specified ZCM URL\n"
                "  -d, --dest-url=URL         Repeat messages onto the specified ZCM URL\n"
                "  -i, --invert-channels      Invert channels. Repeat everything that CHAN\n"
                "                             does not match.\n"
                "\n");
    }
};

struct Repeater
{
    Args args;

    zcm_t *zcmSrc = nullptr;
    zcm_t *zcmDest = nullptr;

    // variables for inverted matching (e.g., logging all but some channels)
    regex invert_regex;

    Repeater() {}

    ~Repeater()
    {
        if (zcmSrc)
            zcm_destroy(zcmSrc);
        if (zcmDest)
            zcm_destroy(zcmDest);
    }

    bool init(int argc, char *argv[])
    {
        if (!args.parse(argc, argv))
            return false;

        // Source network
        zcmSrc = zcm_create(args.src_url.c_str());
        if (!zcmSrc) {
            fprintf(stderr, "Couldn't initialize source ZCM! "
                            "Please check your source transport url.\n\n");
            return false;
        }

        // Dest network
        zcmDest = zcm_create(args.dest_url.c_str());
        if (!zcmDest) {
            fprintf(stderr, "Couldn't initialize destination ZCM! "
                            "Please check your destination transport url.\n\n");
            return false;
        }

        // Compile the regex if we are in invert mode
        if (args.invert_channels) {
            invert_regex = regex{args.chan};
        }

        return true;
    }

    const char *getSubChannel()
    {
        // if inverting the channels, subscribe to everything and invert on the callback
        return (!args.invert_channels) ? args.chan.c_str() : ".*";
    }

    static void handler(const zcm_recv_buf_t *rbuf, const char *channel, void *usr)
    { ((Repeater*)usr)->handler_(rbuf, channel); }

    void handler_(const zcm_recv_buf_t *rbuf, const char *channel)
    {
        if (args.invert_channels) {
            cmatch match;
            regex_match(channel, match, invert_regex);
            if (match.size() > 0)
                return;
        }

        zcm_publish(zcmDest, channel, rbuf->data, rbuf->data_size);
    }

    void run()
    {
        zcm_subscribe(zcmSrc, getSubChannel(), &handler, this);
        zcm_run(zcmSrc);
    }
};

int main(int argc, char *argv[])
{
    Repeater repeater{};
    if (!repeater.init(argc, argv)) return 1;

    repeater.run();

    fprintf(stderr, "Repeater exiting\n");

    return 0;
}
