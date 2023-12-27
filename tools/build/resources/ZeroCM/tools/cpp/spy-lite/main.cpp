#include <getopt.h>

#include "util/TypeDb.hpp"

#include "Common.hpp"
#include "MsgDisplay.hpp"
#include "MsgInfo.hpp"
#include "Debug.hpp"

#define SELECT_TIMEOUT 20000
#define ESCAPE_KEY 0x1B
#define DEL_KEY 0x7f

static volatile bool quit = false;

enum class DisplayMode {
    Overview, Decode, Help
};

struct SpyInfo
{
    SpyInfo(const char *path, bool showBandwidth, bool debug)
    : typedb(path, debug), showBandwidth(showBandwidth)
    {
    }

    ~SpyInfo()
    {
        for (auto& it : minfomap)
            delete it.second;
    }

    bool good() const
    {
        return typedb.good();
    }

    MsgInfo *getCurrentMsginfo(const char **channel)
    {
        string ch;

        assert(decode_index >= 0);

        size_t num = 0;
        for (const auto& channel : names) {
            if (!prefix_filter.empty() && channel.rfind(prefix_filter, 0) != 0)
                continue;
            if (num == (size_t)decode_index) {
                ch = channel;
                break;
            }
            num++;
        }
        assert(!ch.empty());
        MsgInfo **m = lookup(minfomap, ch);
        assert(m);
        if (channel)
            *channel = ch.c_str();
        return *m;
    }

    bool isValidChannelnum(size_t index)
    {
        size_t num = 0;
        for (const auto& channel : names) {
            if (!prefix_filter.empty() && channel.rfind(prefix_filter, 0) != 0)
                continue;
            num++;
        }
        return index < num;
    }

    void addMessage(const char *channel, const zcm_recv_buf_t *rbuf)
    {
        unique_lock<mutex> lk(mut);

        MsgInfo *minfo = minfomap[channel];
        if (minfo == NULL) {
            minfo = new MsgInfo(typedb, channel);
            names.push_back(channel);
            std::sort(begin(names), end(names));
            minfomap[channel] = minfo;
        }
        minfo->addMessage(TimeUtil::utime(), rbuf);
    }

    void display()
    {
        unique_lock<mutex> lk(mut);

        switch (mode) {
            case DisplayMode::Overview: {
                displayOverview();
            } break;
            case DisplayMode::Decode: {
                decode_msg_info->display();

                if (is_selecting) {
                    printf("   Decode field: ");
                    if (view_id != -1)
                        printf("%d", view_id );
                    fflush(stdout);
                }
            } break;
            case DisplayMode::Help: {
                displayHelp();
            } break;
            default:
                DEBUG(1, "ERR: unknown mode\n");
        }
    }

    void displayOverview()
    {
        if (showBandwidth) {
            printf("         %-31s%12s    %8s   %4s\n", "Channel", "Num Messages", "Hz (avg)", "KBps (avg)");
            printf("   -----------------------------------------------------------------------------\n");
        } else {
            printf("         %-31s%12s    %8s\n", "Channel", "Num Messages", "Hz (avg)");
            printf("    ----------------------------------------------------------------\n");
        }

        DEBUG(5, "start-loop\n");

        size_t numShowing = 0;
        for (size_t i = 0; i < names.size(); i++) {
            auto& channel = names[i];
            if (!prefix_filter.empty() && channel.rfind(prefix_filter, 0) != 0)
                continue;
            MsgInfo **minfo = lookup(minfomap, channel);
            assert(minfo != NULL);
            float hz = (*minfo)->getHertz();
            printf("   %3zu)  %-31s%12" PRIu64 "    %7.2f",
                   numShowing, channel.c_str(), (*minfo)->getNumMsgs(), hz);
            if (showBandwidth) {
                float bandwidth = (*minfo)->getBandwidthBps() / 1024;
                printf("     %7.2f", bandwidth);
            }
            printf("\n");
            numShowing++;
        }

        printf("\n");

        if (!prefix_filter.empty() || is_setting_prefix) {
            printf("   Prefix: %s", prefix_filter.c_str());
            if (is_selecting || !is_setting_prefix) printf("\n");
            else fflush(stdout);
        }
        if (is_selecting) {
            printf("   Decode channel: ");
            if (decode_index != -1)
                printf("%d", decode_index);
            fflush(stdout);
        }
    }

    void displayHelp()
    {
        printf("\n"
               "    Terminal based spy utility.  Subscribes to all channels on a ZCM\n"
               "    transport and displays them in an interactive terminal.\n"
               "\n"
               "    Press Key:\n"
               "\n"
               "       -             Allows for entering multi-digit field\n"
               "       %%             Allows for entering channel filter prefix\n"
               "       [a-z,A-Z,_]   Immediately begin typing channel filter prefix\n"
               "       ESC           Exit help menu\n"
               "\n");
    }

    void handleKeyboardOverviewSettingPrefix(char ch)
    {
        if (ch == ESCAPE_KEY) {
            prefix_filter.clear();
            is_setting_prefix = false;
        } else if (ch == '\b' || ch == DEL_KEY) {
            if (!prefix_filter.empty()) prefix_filter.pop_back();
        } else if (ch == '\n') {
            is_setting_prefix = false;
        } else {
            prefix_filter = prefix_filter + ch;
        }
    }

    void handleKeyboardOverviewSelecting(char ch)
    {
        if (ch == '-') {
            is_selecting = true;
            decode_index = -1;
        } else if ('0' <= ch && ch <= '9') {
            if (decode_index == -1) {
                decode_index = ch - '0';
            } else if (decode_index < 10000) {
                decode_index *= 10;
                decode_index += (ch - '0');
            }
        } else if (ch == '\n') {
            if (isValidChannelnum(decode_index)) {
                decode_msg_info = getCurrentMsginfo(&decode_msg_channel);
                mode = DisplayMode::Decode;
            }
            is_selecting = false;
        } else if (ch == ESCAPE_KEY) {
            is_selecting = false;
        } else if (ch == '\b' || ch == DEL_KEY) {
            if (decode_index < 10)
                decode_index = -1;
            else
                decode_index /= 10;
        }
    }

    void handleKeyboardOverview(char ch)
    {
        if (is_selecting) {
            handleKeyboardOverviewSelecting(ch);
        } else if (is_setting_prefix) {
            handleKeyboardOverviewSettingPrefix(ch);
        } else {
            if (ch == '-') {
                is_selecting = true;
                decode_index = -1;
            } else if (ch == '%') {
                is_setting_prefix = true;
            } else if (ch == '?') {
                prev_mode = DisplayMode::Overview;
                mode = DisplayMode::Help;
            } else if (ch == ESCAPE_KEY) {
                prefix_filter.clear();
            } else if (('a' <= ch && ch <= 'z') ||
                       ('A' <= ch && ch <= 'Z') ||
                       (ch == '_') || (ch == '/')) {
                is_setting_prefix = true;
                return handleKeyboardOverviewSettingPrefix(ch);
            } else if ('0' <= ch && ch <= '9') {
                // shortcut for single digit channels
                decode_index = ch - '0';
                if (isValidChannelnum(decode_index)) {
                    decode_msg_info = getCurrentMsginfo(&decode_msg_channel);
                    mode = DisplayMode::Decode;
                }
            } else {
                DEBUG(1, "INFO: unrecognized input: '%c' (0x%2x)\n", ch, ch);
            }
        }
    }

    void handleKeyboardDecode(char ch)
    {
        MsgInfo& minfo = *decode_msg_info;
        size_t depth = minfo.getViewDepth();

        if (ch == ESCAPE_KEY) {
            if (depth > 0) {
                minfo.decViewDepth();
            } else {
                mode = DisplayMode::Overview;
            }
            is_selecting = false;
        } else if (ch == '-') {
            is_selecting = true;
            view_id = -1;
        } else if ('0' <= ch && ch <= '9') {
            // shortcut for single digit channels
            if (!is_selecting) {
                view_id = ch - '0';
                // set and increase sub-msg decoding depth
                minfo.incViewDepth(view_id);
            } else {
                if (view_id == -1) {
                    view_id = ch - '0';
                } else if (view_id < 10000) {
                    view_id *= 10;
                    view_id += (ch - '0');
                }
            }
        } else if (ch == '\n') {
            if (is_selecting) {
                // set and increase sub-msg decoding depth
                minfo.incViewDepth(view_id);
                is_selecting = false;
            }
        } else if (ch == '\b' || ch == DEL_KEY) {
            if (is_selecting) {
                if (view_id < 10)
                    view_id = -1;
                else
                    view_id /= 10;
            }
        } else if (ch == '?') {
            prev_mode = DisplayMode::Decode;
            mode = DisplayMode::Help;
        } else {
            DEBUG(1, "INFO: unrecognized input: '%c' (0x%2x)\n", ch, ch);
        }
    }

    void handleKeyboardHelp(char ch)
    {
        if (ch == ESCAPE_KEY || ch == 'q') {
            mode = prev_mode;
        } else {
            DEBUG(1, "INFO: unrecognized input: '%c' (0x%2x)\n", ch, ch);
        }
    }

    void handleKeyboard(char ch)
    {
        unique_lock<mutex> lk(mut);

        switch (mode) {
            case DisplayMode::Overview: handleKeyboardOverview(ch); break;
            case DisplayMode::Decode:   handleKeyboardDecode(ch);  break;
            case DisplayMode::Help:     handleKeyboardHelp(ch);  break;
            default:
                DEBUG(1, "INFO: unrecognized keyboard mode: %d\n", (int)mode);
        }
    }

    vector<string>                  names;
    unordered_map<string, MsgInfo*> minfomap;
    TypeDb typedb;

    mutex mut;

    bool showBandwidth;

    DisplayMode mode = DisplayMode::Overview;
    DisplayMode prev_mode = mode;
    bool is_selecting = false;

    int decode_index = 0;
    MsgInfo *decode_msg_info;
    const char *decode_msg_channel;
    int view_id;

    string prefix_filter = "";
    bool is_setting_prefix = false;
};

void *keyboard_thread_func(void *arg)
{
    SpyInfo *spy = (SpyInfo *)arg;

    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");

    struct termios newt = old;
    newt.c_lflag &= ~ICANON;
    newt.c_lflag &= ~ECHO;
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &newt) < 0)
        perror("tcsetattr ICANON");

    char ch[3];
    while (!quit) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(0, &fds);

        struct timeval timeout = { 0, SELECT_TIMEOUT };
        int status = select(1, &fds, 0, 0, &timeout);

        if (quit) break;

        if (status != 0 && FD_ISSET(0, &fds)) {

            int ret = read(0, &ch, sizeof(ch) / sizeof(char));
            if (ret < 0) {
                perror ("read()");
                continue;
            }
            if (ret != 1) {
                // treating all special characters as escape
                ch[0] = ESCAPE_KEY;
            }
            spy->handleKeyboard(ch[0]);

        } else {
            DEBUG(4, "INFO: keyboard_thread_func select() timeout\n");
        }
    }

    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror ("tcsetattr ~ICANON");

    return NULL;
}

void clearscreen()
{
    // clear
    printf("\033[2J");

    // move cursor to (0, 0)
    printf("\033[0;0H");
}

void printThreadFunc(SpyInfo *spy)
{
    static constexpr float hz = 20.0;
    static constexpr u64 period = 1000000 / hz;

    DEBUG(1, "INFO: %s: Starting\n", "print_thread");
    while (!quit) {
        usleep(period);

        clearscreen();
        if (spy->showBandwidth) {
            printf("  ******************************************************************************* \n");
            printf("  ******************************** ZCM-SPY-LITE ********************************* \n");
            printf("  ******************************************************************************* \n");
        } else {
            printf("  **************************************************************************** \n");
            printf("  ******************************* ZCM-SPY-LITE ******************************* \n");
            printf("  **************************************************************************** \n");
        }

        spy->display();

        // flush the stdout buffer (required since we use full buffering)
        fflush(stdout);
    }

    DEBUG(1, "INFO: %s: Ending\n", "print_thread");
}

void handler_all_zcm (const zcm_recv_buf_t *rbuf,
                      const char *channel, void *arg)
{
    SpyInfo *spy = (SpyInfo *)arg;
    spy->addMessage(channel, rbuf);
}

static void sighandler(int s)
{
    switch(s) {
        case SIGQUIT:
        case SIGINT:
        case SIGTERM:
            DEBUG(1, "Caught signal...\n");
            quit = true;
            break;
        default:
            DEBUG(1, "WRN: unrecognized signal fired\n");
            break;
    }
}

struct Args
{
    const char *zcmurl = nullptr;
    const char *zcmtypes_path = nullptr;
    vector<string> channels;
    bool debug = false;
    bool showBandwidth = false;

    bool parse(int argc, char *argv[])
    {
        // set some defaults
        const char *optstring = "hu:p:c:bd";
        struct option long_opts[] = {
            { "help",            no_argument, 0, 'h' },
            { "zcm-url",   required_argument, 0, 'u' },
            { "type-path", required_argument, 0, 'p' },
            { "channel",   required_argument, 0, 'c' },
            { "bandwidth",       no_argument, 0, 'b' },
            { "debug",           no_argument, 0, 'd' },
            { 0, 0, 0, 0 }
        };

        int c;
        while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0) {
            switch (c) {
                case 'u': zcmurl        = optarg; break;
                case 'p': zcmtypes_path = optarg; break;
                case 'c': channels.push_back(optarg); break;
                case 'b': showBandwidth = true;   break;
                case 'd': debug         = true;   break;
                case 'h': default: usage(); return false;
            };
        }

        if (channels.empty()) channels.push_back(".*");

        return true;
    }

    void usage()
    {
        fprintf(stderr, "usage: zcm-spy-lite [options]\n"
                "\n"
                "    Terminal based spy utility.  Subscribes to all channels on a ZCM\n"
                "    transport and displays them in an interactive terminal.\n"
                "Example:\n"
                "    zcm-spy-lite -u udpm://239.255.76.67:7667 -p path/to/zcmtypes.so\n"
                "\n"
                "Options:\n"
                "\n"
                "  -h, --help                 Shows this help text and exits\n"
                "  -u, --zcm-url=URL          Log messages on the specified ZCM URL\n"
                "  -p, --type-path=PATH       Path to a shared library containing the zcmtypes\n"
                "  -c, --channel=CHANNEL      Channel to subscribe to. Can be specified more than once\n"
                "  -b, --bandwidth            Calculate and show bandwidth of each channel\n"
                "  -d, --debug                Run a dry run to ensure proper spy setup\n"
                "\n");
    }
};

int main(int argc, char *argv[])
{
    DEBUG_INIT();

    Args args;
    if (!args.parse(argc, argv)) return 1;

    // Get path to zcmtypes.so from args if defined; otherwise from $ZCM_SPY_LITE_PATH
    const char *spy_lite_path = args.zcmtypes_path ? args.zcmtypes_path : getenv("ZCM_SPY_LITE_PATH");
    if (args.debug)
        printf("zcm_spy_lite_path='%s'\n", spy_lite_path);
    if (spy_lite_path == NULL) {
        fprintf(stderr, "ERR: zcmtypes.so path not set! Try using -p PATH or set $ZCM_SPY_LITE_PATH  \n");
        fflush(stderr);
        return 1;
    }

    SpyInfo spy {spy_lite_path, args.showBandwidth, args.debug};
    if (!spy.good()) {
        fprintf(stderr, "ERR: Failed to load all specified zcmtype libs\n");
        fflush(stderr);
        exit(1);
    }
    if (args.debug) exit(0);

    signal(SIGINT, sighandler);
    signal(SIGQUIT, sighandler);
    signal(SIGTERM, sighandler);

    // configure stdout buffering: use FULL buffering to avoid flickering
    setvbuf(stdout, NULL, _IOFBF, 2048);

    // start zcm
    zcm_t *zcm = zcm_create(args.zcmurl);
    if (!zcm) {
        fprintf(stderr, "Couldn't initialize ZCM! Try providing a URL with the "
                        "-u opt or setting the ZCM_DEFAULT_URL envvar\n");
        return 1;
    }
    for (auto channel : args.channels) {
        zcm_subscribe(zcm, channel.c_str(), handler_all_zcm, &spy);
    }
    zcm_start(zcm);

    thread printThread {printThreadFunc, &spy};

    // use this thread as the keyboard thread
    keyboard_thread_func(&spy);

    // cleanup
    printThread.join();
    zcm_stop(zcm);
    zcm_destroy(zcm);

    DEBUG(1, "Exiting...\n");
    return 0;
}
