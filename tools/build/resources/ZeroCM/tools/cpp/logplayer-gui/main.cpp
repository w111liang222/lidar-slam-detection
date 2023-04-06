#include <atomic>
#include <cctype>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <getopt.h>
#include <gtk/gtk.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <map>

#include <zcm/zcm-cpp.hpp>

#include "util/TimeUtil.hpp"
#include "util/StringUtil.hpp"
#include "zcm/util/Filter.hpp"

using namespace std;

static atomic_int done {0};

#if GTK_MINOR_VERSION < (22)
static void gtk_menu_popup_at_pointer(GtkMenu *menu, GdkEvent *event)
{
    assert(event->type == GDK_BUTTON_PRESS);
    GdkEventButton *bevent = (GdkEventButton *) event;
    gtk_menu_popup(menu, NULL, NULL, NULL, NULL, bevent->button, bevent->time);
}
#endif

static double mathMap(double a, double inMin, double inMax, double outMin, double outMax)
{
    return (a - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

struct Args
{
    string filename = "";
    string zcmUrlOut = "";
    bool highAccuracyMode = false;
    bool verbose = false;
    bool exitWhenDone = false;
    bool playOnStart = false;

    bool init(int argc, char *argv[])
    {
        struct option long_opts[] = {
            { "help",                no_argument, 0, 'h' },
            { "zcm-url",       required_argument, 0, 'u' },
            { "high-accuracy",       no_argument, 0, 'a' },
            { "verbose",             no_argument, 0, 'v' },
            { "exit-when-done",      no_argument, 0, 'e' },
            { "play-on-start",       no_argument, 0, 'p' },
            { 0, 0, 0, 0 }
        };

        int c;
        while ((c = getopt_long(argc, argv, "hu:avep", long_opts, 0)) >= 0) {
            switch (c) {
                case 'u':        zcmUrlOut = string(optarg);       break;
                case 'a': highAccuracyMode = true;                 break;
                case 'v':          verbose = true;                 break;
                case 'e':     exitWhenDone = true;                 break;
                case 'p':      playOnStart = true;                 break;
                case 'h': default: usage(); return false;
            };
        }

        if (optind == argc - 1)
            filename = string(argv[optind]);

        return true;
    }

    void usage()
    {
        cerr << "usage: zcm-logplayer-gui [options] [FILE]" << endl
             << "" << endl
             << "    Reads packets from an ZCM log file and publishes them to a " << endl
             << "    ZCM transport with a GUI for control of playback." << endl
             << "" << endl
             << "Options:" << endl
             << "" << endl
             << "  -u, --zcm-url=URL      Play logged messages on the specified ZCM URL." << endl
             << "  -a, --high-accuracy    Enable extremely accurate publish timing." << endl
             << "                         Note that enabling this feature will probably consume" << endl
             << "                         a full CPU so logplayer can bypass the OS scheduler" << endl
             << "                         with busy waits in between messages." << endl
             << "  -v, --verbose          Print information about each packet." << endl
             << "  -h, --help             Shows some help text and exits." << endl
             << endl;
    }
};

enum {
   LOG_CHAN_COLUMN,
   PLAY_CHAN_COLUMN,
   ENABLED_COLUMN,
   NUM_COLUMNS
};

struct LogPlayer
{
    Args args;
    zcm::LogFile *zcmIn  = nullptr;
    zcm::ZCM     *zcmOut = nullptr;

    mutex windowLk;
    GtkWidget *window;
    GtkWidget *lblLogName;
    GtkWidget *btnPlay;
    GtkWidget *btnStep;
    GtkWidget *btnSlower;
    GtkWidget *lblSpeedTarget;
    GtkWidget *btnFaster;
    GtkWidget *lblCurrTime;
    GtkWidget *lblCurrSpeed;
    GtkWidget *sclMacroScrub;
    GtkWidget *sclMicroScrub;
    GtkWidget *menuScrub;
    GtkWidget *tblData;
    GtkWidget *btnToggle;
    GtkWidget *txtPrefix;

    uint64_t totalTimeUs;
    uint64_t firstMsgUtime;

    double microScrubCurr = 0;
    double microScrubMin = 0;
    double microScrubMax = 1;
    bool microScrubWasPlayingOnStart;
    bool microScrubIsDragging = false;
    bool macroScrubIsDragging = false;

    bool bDown = false;

    mutex redrawLk;
    condition_variable redrawCv;
    bool redrawNow;

    mutex zcmLk;
    condition_variable zcmCv;
    bool isPlaying = false;
    double speedTarget = 1.0;
    double currSpeed = 0.0;
    int ignoreMicroScrubEvts = 0;
    int ignoreMacroScrubEvts = 0;
    uint64_t microPivotTimeUs;
    uint64_t currMsgUtime = 0;
    uint64_t requestTimeUs = numeric_limits<uint64_t>::max();
    bool stepRequest = false;
    string stepPrefix;

    static constexpr int GDK_LEFT_CLICK = 1;
    static constexpr int GDK_RIGHT_CLICK = 3;

    enum class BookmarkType
    {
        PLAIN,
        LREPEAT,
        RREPEAT,
        NUM_BOOKMARK_TYPES
    };
    static BookmarkType bookmarkTypeFromSting(const string& type)
    {
        if (type == "PLAIN") return BookmarkType::PLAIN;
        if (type == "LREPEAT") return BookmarkType::LREPEAT;
        if (type == "RREPEAT") return BookmarkType::RREPEAT;
        return BookmarkType::NUM_BOOKMARK_TYPES;
    }
    static string bookmarkTypeToSting(BookmarkType type)
    {
        switch (type) {
            case BookmarkType::PLAIN: return "PLAIN";
            case BookmarkType::LREPEAT: return "LREPEAT";
            case BookmarkType::RREPEAT: return "RREPEAT";
            case BookmarkType::NUM_BOOKMARK_TYPES: assert(false && "Shouldn't be possible");
        }
        assert(false && "Shouldn't be possible");
        return "";
    }
    struct Bookmark
    {
        BookmarkType type;
        double logTimePerc;
        bool addedToGui;
        Bookmark(BookmarkType type, double logTimePerc) :
            type(type), logTimePerc(logTimePerc), addedToGui(false)
        {}
    };
    vector<Bookmark> bookmarks;

    struct ChannelInfo
    {
        string pubChannel;
        bool enabled;
        bool addedToGui;
    };
    map<string, ChannelInfo> channelMap;

    LogPlayer() {}

    ~LogPlayer()
    {
        if (zcmIn)  { delete zcmIn;  }
        if (zcmOut) { delete zcmOut; }
    }

    void loadPreferences()
    {
        string jlpPath = args.filename + ".jlp";

        fstream jlpFile;
        jlpFile.open(jlpPath.c_str(), ios::in);
        if (!jlpFile.is_open()) return; // no jlp file is fine

        string line;
        while (getline(jlpFile, line)) {
            auto toks = StringUtil::split(line, ' ');
            if (toks.size() < 1) continue;
            if (toks[0] == "BOOKMARK") {
                if (toks.size() < 3) continue;
                BookmarkType type = bookmarkTypeFromSting(toks[1]);
                if (type == BookmarkType::NUM_BOOKMARK_TYPES) continue;
                char *endptr;
                double logTimePerc = std::strtod(toks[2].c_str(), &endptr);
                if (endptr == toks[2].c_str()) continue;
                addBookmark(type, logTimePerc);
            } else if (toks[0] == "CHANNEL") {
                if (toks.size() < 4) continue;
                channelMap[toks[1]].pubChannel = toks[2];
                channelMap[toks[1]].enabled = string(toks[3]) == "true";
            } else {
                cerr << "Unsupported JLP directive: " << line << endl;
            }
        }
        jlpFile.close();
    }

    void savePreferences()
    {
        string jlpPath = args.filename + ".jlp";

        ofstream jlpFile;
        jlpFile.open(jlpPath.c_str(), ios::out);
        if (!jlpFile.is_open()) return; // no jlp file is fine

        map<string, ChannelInfo> _channelMap;
        {
            unique_lock<mutex> lk(zcmLk);
            _channelMap = channelMap;
        }

        for (const auto& b : bookmarks) {
            jlpFile << "BOOKMARK "
                    << bookmarkTypeToSting(b.type) << " "
                    << std::setprecision(std::numeric_limits<long double>::digits10 + 1)
                    << std::fixed << b.logTimePerc << endl;
        }

        for (const auto& c : _channelMap) {
            jlpFile << "CHANNEL "
                    << c.first << " "
                    << c.second.pubChannel << " "
                    << (c.second.enabled ? "true" : "false") << endl;
        }

        jlpFile.close();
    }

    void wakeup()
    {
        zcmCv.notify_all();
        redrawCv.notify_all();
    }

    void addBookmark(BookmarkType type, double logTimePerc)
    {
        bookmarks.emplace_back(type, logTimePerc);
        {
            unique_lock<mutex> lk(windowLk);
            if (window) {
                gtk_scale_add_mark(GTK_SCALE(sclMacroScrub),
                                   bookmarks.back().logTimePerc, GTK_POS_TOP,
                                   to_string(bookmarks.size() - 1).c_str());
                bookmarks.back().addedToGui = true;
            }
        }
    }

    bool toggleChannelPublish(GtkTreeIter iter)
    {
        GtkTreeModel *model = gtk_tree_view_get_model(GTK_TREE_VIEW(tblData));
        gchar *name;
        gtk_tree_model_get(model, &iter, LOG_CHAN_COLUMN, &name, -1);

        bool enabled;
        {
            unique_lock<mutex> lk(zcmLk);
            channelMap[name].enabled = !channelMap[name].enabled;
            enabled = channelMap[name].enabled;
        }
        savePreferences();

        gtk_list_store_set(GTK_LIST_STORE(model), &iter, ENABLED_COLUMN, enabled, -1);

        return enabled;
    }

    bool initializeZcm()
    {
        if (args.filename.empty()) return false;

        zcmIn = new zcm::LogFile(args.filename, "r");
        if (!zcmIn->good()) {
            delete zcmIn;
            zcmIn = nullptr;
            cerr << "Error: Failed to open '" << args.filename << "'" << endl;
            return false;
        }

        const zcm::LogEvent* leStart = zcmIn->readNextEvent();
        if (!leStart) {
            delete zcmIn;
            zcmIn = nullptr;
            cerr << "Error: Unable to find valid event in logfile: '" << args.filename << "'" << endl;
            return false;
        }
        firstMsgUtime = (uint64_t)leStart->timestamp;
        currMsgUtime = firstMsgUtime;

        zcmOut = new zcm::ZCM(args.zcmUrlOut);
        if (!zcmOut->good()) {
            delete zcmOut;
            zcmOut = nullptr;
            delete zcmIn;
            zcmIn = nullptr;
            cerr << "Error: Failed to open zcm: '" << args.zcmUrlOut << "'" << endl;
            return false;
        }

        fseeko(zcmIn->getFilePtr(), 0, SEEK_END);

        const zcm::LogEvent* leEnd = zcmIn->readPrevEvent();
        if (!leEnd) leEnd = zcmIn->readPrevEvent(); // In case log was cut off at end
        assert(leEnd);

        uint64_t lastMsgUtime = (uint64_t)leEnd->timestamp;

        assert(lastMsgUtime > firstMsgUtime);
        totalTimeUs = lastMsgUtime - firstMsgUtime;

        loadPreferences();

        return true;
    }

    bool init(int argc, char *argv[])
    {
        if (!args.init(argc, argv))
            return false;

        isPlaying = args.playOnStart;

        {
            unique_lock<mutex> lk(zcmLk);
            initializeZcm();
        }

        return true;
    }

    void enableUI(gboolean enable)
    {
        gtk_widget_set_sensitive(btnPlay, enable);
        gtk_widget_set_sensitive(btnStep, enable);
        gtk_widget_set_sensitive(btnSlower, enable);
        gtk_widget_set_sensitive(btnFaster, enable);
        gtk_widget_set_sensitive(sclMacroScrub, enable);
        gtk_widget_set_sensitive(sclMicroScrub, enable);
        gtk_widget_set_sensitive(tblData, enable);
        gtk_widget_set_sensitive(btnToggle, enable);
        gtk_widget_set_sensitive(txtPrefix, enable);
    }

    void addChannels(const map<string, ChannelInfo>& channelMap)
    {
        GtkTreeModel *model = gtk_tree_view_get_model(GTK_TREE_VIEW(tblData));

        for (const auto& c : channelMap) {
            if (c.second.addedToGui) continue;
            GtkTreeIter iter;
            gtk_list_store_append(GTK_LIST_STORE(model), &iter);
            gtk_list_store_set(GTK_LIST_STORE(model), &iter,
                               LOG_CHAN_COLUMN, c.first.c_str(), -1);
            gtk_list_store_set(GTK_LIST_STORE(model), &iter,
                               PLAY_CHAN_COLUMN, c.second.pubChannel.c_str(), -1);
            gtk_list_store_set(GTK_LIST_STORE(model), &iter,
                               ENABLED_COLUMN, c.second.enabled, -1);
        }
    }

    static gboolean zcmUpdateGui(LogPlayer *me)
    {
        map<string, ChannelInfo> _channelMap;
        double currSpeed;
        double currTimeS;
        double totalTimeS;
        double perc;
        bool isPlaying;
        {
            unique_lock<mutex> lk(me->zcmLk);

            perc = (double) (me->currMsgUtime - me->firstMsgUtime) / (double) me->totalTimeUs;

            _channelMap = me->channelMap;
            for (auto& c : me->channelMap) c.second.addedToGui = true;

            currTimeS = (me->currMsgUtime - me->firstMsgUtime) / 1e6;

            totalTimeS = me->totalTimeUs / 1e6;

            currSpeed = me->currSpeed;

            isPlaying = me->isPlaying;
        }
        {
            unique_lock<mutex> lk(me->windowLk);

            if (!me->window) return FALSE;

            auto changeIgnore = [](GtkRange *range, int &ignoreCount, double val){
                double lastVal = gtk_range_get_value(range);
                if (val != lastVal) {
                    ignoreCount++;
                    gtk_range_set_value(range, val);
                }
            };

            if (!me->macroScrubIsDragging)
                changeIgnore(GTK_RANGE(me->sclMacroScrub), me->ignoreMacroScrubEvts, perc);

            if (currTimeS < 1) {
                me->microScrubMin = -currTimeS;
                me->microScrubMax = 1;
                double displayVal = mathMap(me->microScrubCurr,
                                       me->microScrubMin, me->microScrubMax, 0, 1);
                if (!me->microScrubIsDragging)
                    changeIgnore(GTK_RANGE(me->sclMicroScrub),
                                 me->ignoreMicroScrubEvts, displayVal);
            } else if (currTimeS > totalTimeS - 1) {
                me->microScrubMin = -1;
                me->microScrubMax = totalTimeS - currTimeS;
                double displayVal = mathMap(me->microScrubCurr,
                                       me->microScrubMin, me->microScrubMax, 0, 1);
                if (!me->microScrubIsDragging)
                    changeIgnore(GTK_RANGE(me->sclMicroScrub),
                                 me->ignoreMicroScrubEvts, displayVal);
            } else {
                me->microScrubMin = -1;
                me->microScrubMax = 1;
                double displayVal = mathMap(me->microScrubCurr,
                                       me->microScrubMin, me->microScrubMax, 0, 1);
                if (!me->microScrubIsDragging)
                    changeIgnore(GTK_RANGE(me->sclMicroScrub),
                                 me->ignoreMicroScrubEvts, displayVal);
            }

            gchar buf[50];

            g_snprintf(buf, 50, "%9.3f s / %.3f s", currTimeS, totalTimeS);
            gtk_label_set_text(GTK_LABEL(me->lblCurrTime), buf);

            g_snprintf(buf, 50, "%.2f x", currSpeed);
            gtk_label_set_text(GTK_LABEL(me->lblCurrSpeed), buf);

            if (isPlaying) {
                gtk_button_set_label(GTK_BUTTON(me->btnPlay), "Pause");
            } else {
                gtk_button_set_label(GTK_BUTTON(me->btnPlay), "Play");
            }

            me->addChannels(_channelMap);
        }

        return FALSE;
    }

    void updateSpeedTarget()
    {
        gchar speedStr[20];
        g_snprintf(speedStr, 20, "%.3f", speedTarget);
        gtk_label_set_text(GTK_LABEL(lblSpeedTarget), speedStr);
    }

    static void prefixChanged(GtkEditable *editable, LogPlayer *me)
    {
        const gchar *prefix;
        prefix = gtk_entry_get_text(GTK_ENTRY(editable));
        {
            unique_lock<mutex> lk(me->zcmLk);
            me->stepPrefix = prefix;
            me->stepRequest = false;
            me->isPlaying = false;
        }
    }

    static void toggle(GtkWidget *widget, LogPlayer *me)
    {
        auto toggleChan = [](GtkTreeModel *model, GtkTreePath *path,
                             GtkTreeIter *iter, gpointer usr) {
            LogPlayer *me = (LogPlayer*) usr;
            me->toggleChannelPublish(*iter);
        };

        GtkTreeSelection *selection = gtk_tree_view_get_selection(GTK_TREE_VIEW(me->tblData));
        gtk_tree_selection_selected_foreach(selection, toggleChan, me);
    }

    static void playbackChanEdit(GtkCellRendererText *cell,
                                 gchar *path, gchar *newChan,
                                 LogPlayer *me)
    {
        GtkTreeModel *model = gtk_tree_view_get_model(GTK_TREE_VIEW(me->tblData));

        GtkTreeIter iter;
        gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(model), &iter, path);

        gchar *chan;
        gtk_tree_model_get(GTK_TREE_MODEL(model), &iter, LOG_CHAN_COLUMN, &chan, -1);
        gtk_list_store_set(GTK_LIST_STORE(model), &iter, PLAY_CHAN_COLUMN, newChan, -1);
        {
            unique_lock<mutex> lk(me->zcmLk);
            me->channelMap[chan].pubChannel = newChan;
        }
        me->savePreferences();
    }

    static void channelEnable(GtkCellRendererToggle *cell, gchar *path, LogPlayer *me)
    {
        GtkTreeModel *model = gtk_tree_view_get_model(GTK_TREE_VIEW(me->tblData));

        GtkTreeIter iter;
        gtk_tree_model_get_iter_from_string(GTK_TREE_MODEL(model), &iter, path);

        me->toggleChannelPublish(iter);
    }

    void bookmark()
    {
        double perc;
        {
            unique_lock<mutex> lk(zcmLk);
            perc = (double) (currMsgUtime - firstMsgUtime) / (double) totalTimeUs;
        }
        addBookmark(BookmarkType::PLAIN, perc);
        savePreferences();
    }

    static void bookmarkClicked(GtkWidget *bookmark, LogPlayer *me)
    {
        me->bookmark();
    }

    void requestTime(uint64_t logUtime)
    {
        {
            unique_lock<mutex> lk(zcmLk);
            requestTimeUs = logUtime;
        }
        wakeup();
    }

    static gboolean keyPress(GtkWidget *widget, GdkEvent *event, LogPlayer *me)
    {
        if (event->type == GDK_KEY_PRESS) {

            GdkEventKey *kevent = (GdkEventKey*) event;

            if (kevent->keyval == GDK_KEY_Escape) {
                gtk_window_set_focus(GTK_WINDOW(me->window), me->btnPlay);
                GtkTreeSelection *selection = gtk_tree_view_get_selection(GTK_TREE_VIEW(me->tblData));
                gtk_tree_selection_unselect_all(selection);
            }

            GtkWidget *focusWidget = gtk_window_get_focus(GTK_WINDOW(me->window));
            if (focusWidget == me->txtPrefix || focusWidget == me->tblData)
                return FALSE;

            switch (kevent->keyval) {
                case GDK_KEY_b:
                    if (!me->bDown) {
                        me->bDown = true;
                        me->bookmark();
                    }
                    break;
                case GDK_KEY_KP_0:
                case GDK_KEY_0:
                    if (me->bookmarks.size() <= 0) break;
                    me->requestTime(me->firstMsgUtime +
                                    me->bookmarks[0].logTimePerc * me->totalTimeUs);
                    break;
                case GDK_KEY_KP_1:
                case GDK_KEY_1:
                    if (me->bookmarks.size() <= 1) break;
                    me->requestTime(me->firstMsgUtime +
                                    me->bookmarks[1].logTimePerc * me->totalTimeUs);
                    break;
                case GDK_KEY_KP_2:
                case GDK_KEY_2:
                    if (me->bookmarks.size() <= 2) break;
                    me->requestTime(me->firstMsgUtime +
                                    me->bookmarks[2].logTimePerc * me->totalTimeUs);
                    break;
                case GDK_KEY_KP_3:
                case GDK_KEY_3:
                    if (me->bookmarks.size() <= 3) break;
                    me->requestTime(me->firstMsgUtime +
                                    me->bookmarks[3].logTimePerc * me->totalTimeUs);
                    break;
                case GDK_KEY_KP_4:
                case GDK_KEY_4:
                    if (me->bookmarks.size() <= 4) break;
                    me->requestTime(me->firstMsgUtime +
                                    me->bookmarks[4].logTimePerc * me->totalTimeUs);
                    break;
                case GDK_KEY_KP_5:
                case GDK_KEY_5:
                    if (me->bookmarks.size() <= 5) break;
                    me->requestTime(me->firstMsgUtime +
                                    me->bookmarks[5].logTimePerc * me->totalTimeUs);
                    break;
                case GDK_KEY_KP_6:
                case GDK_KEY_6:
                    if (me->bookmarks.size() <= 6) break;
                    me->requestTime(me->firstMsgUtime +
                                    me->bookmarks[6].logTimePerc * me->totalTimeUs);
                    break;
                case GDK_KEY_KP_7:
                case GDK_KEY_7:
                    if (me->bookmarks.size() <= 7) break;
                    me->requestTime(me->firstMsgUtime +
                                    me->bookmarks[7].logTimePerc * me->totalTimeUs);
                    break;
                case GDK_KEY_KP_8:
                case GDK_KEY_8:
                    if (me->bookmarks.size() <= 8) break;
                    me->requestTime(me->firstMsgUtime +
                                    me->bookmarks[8].logTimePerc * me->totalTimeUs);
                    break;
                case GDK_KEY_KP_9:
                case GDK_KEY_9:
                    if (me->bookmarks.size() <= 9) break;
                    me->requestTime(me->firstMsgUtime +
                                    me->bookmarks[9].logTimePerc * me->totalTimeUs);
                    break;
            }
        } else if (event->type == GDK_KEY_RELEASE) {
            GdkEventKey *kevent = (GdkEventKey*) event;
            switch (kevent->keyval) {
                case GDK_KEY_b:
                    me->bDown = false;
                    break;
            }
        }
        return FALSE;
    }

    static void macroScrub(GtkRange *range, LogPlayer *me)
    {
        if (me->ignoreMacroScrubEvts > 0) {
            me->ignoreMacroScrubEvts--;
            return;
        }
        gdouble pos = gtk_range_get_value(range);
        me->requestTime(me->firstMsgUtime + pos * me->totalTimeUs);
    }

    static gboolean macroScrubClicked(GtkRange *range, GdkEvent *event, LogPlayer *me)
    {
        if (event->type == GDK_BUTTON_PRESS) {
            GdkEventButton *bevent = (GdkEventButton *) event;
            if (bevent->button == GDK_RIGHT_CLICK) {
                gtk_menu_popup_at_pointer(GTK_MENU(me->menuScrub), event);
                return TRUE;
            } else if (bevent->button == GDK_LEFT_CLICK) {
                me->macroScrubIsDragging = true;
            }
        } else if (event->type == GDK_BUTTON_RELEASE) {
            GdkEventButton *bevent = (GdkEventButton *) event;
            if (bevent->button == GDK_LEFT_CLICK) {
                me->macroScrubIsDragging = false;
            }
        }
        return FALSE;
    }

    static void microScrub(GtkRange *range, LogPlayer *me)
    {
        if (me->ignoreMicroScrubEvts > 0) {
            me->ignoreMicroScrubEvts--;
            return;
        }
        gdouble pos = gtk_range_get_value(GTK_RANGE(range));
        pos = mathMap(pos, 0, 1, me->microScrubMin, me->microScrubMax);
        me->requestTime(me->microPivotTimeUs + pos * 1e6);
        return;
    }

    static gboolean microScrubClicked(GtkWidget *range, GdkEvent *event, LogPlayer *me)
    {
        if (event->type == GDK_KEY_PRESS) {
            unique_lock<mutex> lk(me->zcmLk);
            me->microPivotTimeUs = me->currMsgUtime;
        } else if (event->type == GDK_BUTTON_PRESS) {
            GdkEventButton *bevent = (GdkEventButton*) event;
            if (bevent->button == GDK_RIGHT_CLICK) {
                gtk_menu_popup_at_pointer(GTK_MENU(me->menuScrub), event);
                return TRUE;
            } else if (bevent->button == GDK_LEFT_CLICK) {
                me->microScrubIsDragging = true;
                {
                    unique_lock<mutex> lk(me->zcmLk);
                    me->microPivotTimeUs = me->currMsgUtime;
                    me->microScrubWasPlayingOnStart = me->isPlaying;
                    me->isPlaying = false;
                }
            }
        } else if (event->type == GDK_BUTTON_RELEASE) {
            GdkEventButton *bevent = (GdkEventButton *) event;
            if (bevent->button == GDK_LEFT_CLICK) {
                me->microScrubIsDragging = false;
                double pos = mathMap(0, me->microScrubMin, me->microScrubMax, 0, 1);
                me->ignoreMicroScrubEvts++;
                gtk_range_set_value(GTK_RANGE(me->sclMicroScrub), pos);
                {
                    unique_lock<mutex> lk(me->zcmLk);
                    me->isPlaying = me->microScrubWasPlayingOnStart;
                }
                me->wakeup();
            }
        }
        return FALSE;
    }

    static gboolean openLog(GtkWidget *widget, GdkEventButton *event, LogPlayer* me)
    {
        gboolean ret = TRUE;
        if (event->type == GDK_2BUTTON_PRESS) {
            assert(me->args.filename == "");

            GtkWidget *dialog = gtk_file_chooser_dialog_new("Open File",
                                                            GTK_WINDOW(me->window),
                                                            GTK_FILE_CHOOSER_ACTION_OPEN,
                                                            "Cancel",
                                                            GTK_RESPONSE_CANCEL,
                                                            "Open",
                                                            GTK_RESPONSE_ACCEPT,
                                                            (void*)NULL);
            gint res = gtk_dialog_run(GTK_DIALOG(dialog));

            if (res == GTK_RESPONSE_ACCEPT) {
                char *filename;
                GtkFileChooser *chooser = GTK_FILE_CHOOSER(dialog);
                filename = gtk_file_chooser_get_filename(chooser);

                bool zcmEnabled;
                {
                    unique_lock<mutex> lk(me->zcmLk);
                    me->args.filename = string(filename);
                    zcmEnabled = me->initializeZcm();
                    if (!zcmEnabled) me->args.filename = "";
                }

                if (zcmEnabled) {
                    gtk_label_set_text(GTK_LABEL(me->lblLogName),
                                       StringUtil::basename(filename).c_str());
                    me->enableUI(true);
                    me->wakeup();
                    ret = FALSE;
                }

                g_free(filename);
            }

            gtk_widget_destroy(dialog);
        }
        return ret;
    }

    static void playPause(GtkWidget *widget, LogPlayer *me)
    {
        bool isPlaying;
        {
            unique_lock<mutex> lk(me->zcmLk);
            me->isPlaying = !me->isPlaying;
            isPlaying = me->isPlaying;
            me->wakeup();
        }
        if (isPlaying) {
            gtk_button_set_label(GTK_BUTTON(widget), "Pause");
        } else {
            gtk_button_set_label(GTK_BUTTON(widget), "Play");
        }
    }

    static void step(GtkWidget *widget, LogPlayer *me)
    {
        {
            unique_lock<mutex> lk(me->zcmLk);
            me->stepRequest = true;
            me->isPlaying = true;
        }
        me->wakeup();
        gtk_button_set_label(GTK_BUTTON(me->btnPlay), "Pause");
    }

    static void slow(GtkWidget *widget, LogPlayer *me)
    {
        unique_lock<mutex> lk(me->zcmLk);
        me->speedTarget /= 2;
        me->updateSpeedTarget();
    }

    static void fast(GtkWidget *widget, LogPlayer *me)
    {
        unique_lock<mutex> lk(me->zcmLk);
        me->speedTarget *= 2;
        me->updateSpeedTarget();
    }

    static gboolean windowDelete(GtkWidget *widget, GdkEvent *event, LogPlayer *me)
    {
        unique_lock<mutex> lk(me->windowLk);
        me->window = NULL;
        return FALSE;
    }

    static void windowDestroy(GtkWidget *widget, LogPlayer *me)
    {
        unique_lock<mutex> lk(me->windowLk);
        me->window = NULL;
    }

    static void activate(GtkApplication *app, LogPlayer *me)
    {
        unique_lock<mutex> lk(me->windowLk);

        me->window = gtk_application_window_new(app);
        g_signal_connect(me->window, "delete-event", G_CALLBACK(windowDelete), me);
        g_signal_connect(me->window, "destroy", G_CALLBACK(windowDestroy), me);
        gtk_window_set_title(GTK_WINDOW(me->window), "Zcm Log Player");
        gtk_window_set_default_size(GTK_WINDOW(me->window), 450, 275);
        gtk_window_set_position(GTK_WINDOW(me->window), GTK_WIN_POS_MOUSE);
        gtk_container_set_border_width(GTK_CONTAINER(me->window), 1);
        gtk_widget_add_events(me->window, GDK_KEY_PRESS_MASK);
        g_signal_connect(me->window, "key-press-event",
                         G_CALLBACK(keyPress), me);
        g_signal_connect(me->window, "key-release-event",
                         G_CALLBACK(keyPress), me);

        GtkWidget *grid = gtk_grid_new();
        gtk_container_add(GTK_CONTAINER(me->window), grid);
        gtk_grid_set_row_spacing(GTK_GRID(grid), 10);

        GtkWidget *evtLogName = gtk_event_box_new();
        gtk_event_box_set_above_child(GTK_EVENT_BOX(evtLogName), TRUE);
        gtk_grid_attach(GTK_GRID(grid), evtLogName, 0, 0, 1, 1);

        string logName = "Double click to load";
        if (!me->args.filename.empty())
            logName = StringUtil::basename(me->args.filename.c_str()).c_str();
        me->lblLogName = gtk_label_new(logName.c_str());
        gtk_widget_set_hexpand(me->lblLogName, TRUE);
        gtk_widget_set_halign(me->lblLogName, GTK_ALIGN_CENTER);
        if (me->args.filename.empty()) {
            g_signal_connect(G_OBJECT(evtLogName),
                             "button_press_event",
                             G_CALLBACK(openLog), me);
        }
        gtk_container_add(GTK_CONTAINER(evtLogName), me->lblLogName);


        me->btnPlay = gtk_button_new_with_label("Play");
        g_signal_connect(me->btnPlay, "clicked", G_CALLBACK(playPause), me);
        gtk_widget_set_hexpand(me->btnPlay, TRUE);
        gtk_widget_set_halign(me->btnPlay, GTK_ALIGN_FILL);
        gtk_grid_attach(GTK_GRID(grid), me->btnPlay, 1, 0, 1, 1);

        me->btnStep = gtk_button_new_with_label("Step");
        g_signal_connect(me->btnStep, "clicked", G_CALLBACK(step), me);
        gtk_widget_set_hexpand(me->btnStep, TRUE);
        gtk_widget_set_halign(me->btnStep, GTK_ALIGN_FILL);
        gtk_grid_attach(GTK_GRID(grid), me->btnStep, 2, 0, 1, 1);

        me->btnSlower = gtk_button_new_with_label("<<");
        g_signal_connect(me->btnSlower, "clicked", G_CALLBACK(slow), me);
        gtk_widget_set_hexpand(me->btnSlower, TRUE);
        gtk_widget_set_halign(me->btnSlower, GTK_ALIGN_END);
        gtk_grid_attach(GTK_GRID(grid), me->btnSlower, 3, 0, 1, 1);

        me->lblSpeedTarget = gtk_label_new("");
        me->updateSpeedTarget();
        gtk_widget_set_hexpand(me->lblSpeedTarget, TRUE);
        gtk_widget_set_halign(me->lblSpeedTarget, GTK_ALIGN_CENTER);
        gtk_grid_attach(GTK_GRID(grid), me->lblSpeedTarget, 4, 0, 1, 1);

        me->btnFaster = gtk_button_new_with_label(">>");
        g_signal_connect(me->btnFaster, "clicked", G_CALLBACK(fast), me);
        gtk_widget_set_hexpand(me->btnFaster, TRUE);
        gtk_widget_set_halign(me->btnFaster, GTK_ALIGN_START);
        gtk_grid_attach(GTK_GRID(grid), me->btnFaster, 5, 0, 1, 1);

        me->menuScrub = gtk_menu_new();
        GtkWidget *miBookmark = gtk_menu_item_new_with_label("Bookmark");
        gtk_widget_show(miBookmark);
        gtk_menu_shell_append(GTK_MENU_SHELL(me->menuScrub), miBookmark);
        g_signal_connect(miBookmark, "activate", G_CALLBACK(bookmarkClicked), me);

        GtkAdjustment *adjMacroScrub = gtk_adjustment_new(0, 0, 1, 0.01, 0.05, 0);
        me->sclMacroScrub = gtk_scale_new(GTK_ORIENTATION_HORIZONTAL, adjMacroScrub);
        gtk_scale_set_draw_value(GTK_SCALE(me->sclMacroScrub), FALSE);
        gtk_widget_set_hexpand(me->sclMacroScrub, TRUE);
        gtk_widget_set_valign(me->sclMacroScrub, GTK_ALIGN_FILL);
        g_signal_connect(me->sclMacroScrub, "button-press-event",
                         G_CALLBACK(macroScrubClicked), me);
        g_signal_connect(me->sclMacroScrub, "button-release-event",
                         G_CALLBACK(macroScrubClicked), me);
        g_signal_connect(me->sclMacroScrub, "value-changed", G_CALLBACK(macroScrub), me);
        gtk_grid_attach(GTK_GRID(grid), me->sclMacroScrub, 0, 1, 6, 1);

        GtkAdjustment *adjMicroScrub = gtk_adjustment_new(0, 0, 1, 0.01, 0.05, 0);
        me->sclMicroScrub = gtk_scale_new(GTK_ORIENTATION_HORIZONTAL, adjMicroScrub);
        gtk_scale_set_draw_value(GTK_SCALE(me->sclMicroScrub), FALSE);
        gtk_scale_set_has_origin(GTK_SCALE(me->sclMicroScrub), FALSE);
        gtk_widget_set_hexpand(me->sclMicroScrub, TRUE);
        gtk_widget_set_valign(me->sclMicroScrub, GTK_ALIGN_FILL);
        g_signal_connect(me->sclMicroScrub, "button-press-event",
                         G_CALLBACK(microScrubClicked), me);
        g_signal_connect(me->sclMicroScrub, "button-release-event",
                         G_CALLBACK(microScrubClicked), me);
        g_signal_connect(me->sclMicroScrub, "key-press-event",
                         G_CALLBACK(microScrubClicked), me);
        g_signal_connect(me->sclMicroScrub, "value-changed", G_CALLBACK(microScrub), me);
        gtk_grid_attach(GTK_GRID(grid), me->sclMicroScrub, 0, 2, 6, 1);

        me->lblCurrTime = gtk_label_new("0 s");
        gtk_widget_set_hexpand(me->lblCurrTime, TRUE);
        gtk_widget_set_halign(me->lblCurrTime, GTK_ALIGN_START);
        gtk_grid_attach(GTK_GRID(grid), me->lblCurrTime, 0, 3, 2, 1);

        me->lblCurrSpeed = gtk_label_new("1.0x");
        gtk_widget_set_hexpand(me->lblCurrSpeed, TRUE);
        gtk_widget_set_halign(me->lblCurrSpeed, GTK_ALIGN_START);
        gtk_grid_attach(GTK_GRID(grid), me->lblCurrSpeed, 2, 3, 1, 1);

        me->tblData = gtk_tree_view_new();
        gtk_widget_set_vexpand(me->tblData, TRUE);
        gtk_widget_set_valign(me->tblData, GTK_ALIGN_FILL);
        gtk_tree_view_set_headers_visible(GTK_TREE_VIEW(me->tblData), TRUE);

        GtkTreeSelection *selection = gtk_tree_view_get_selection(GTK_TREE_VIEW(me->tblData));
        gtk_tree_selection_set_mode(selection, GTK_SELECTION_MULTIPLE);

        GtkListStore *store = gtk_list_store_new(NUM_COLUMNS,
                                                 G_TYPE_STRING, G_TYPE_STRING, G_TYPE_BOOLEAN);
        gtk_tree_view_set_model(GTK_TREE_VIEW(me->tblData), GTK_TREE_MODEL(store));
        gtk_tree_sortable_set_sort_column_id(GTK_TREE_SORTABLE(store), 0, GTK_SORT_ASCENDING);
        g_object_unref(store);

        GtkCellRenderer *logChanRenderer = gtk_cell_renderer_text_new();
        GtkTreeViewColumn *colLogChan =
            gtk_tree_view_column_new_with_attributes("Log Channel", logChanRenderer,
                                                     "text", LOG_CHAN_COLUMN, (void*)NULL);
        gtk_tree_view_column_set_expand(colLogChan, TRUE);
        gtk_tree_view_column_set_resizable(colLogChan, TRUE);
        gtk_tree_view_append_column(GTK_TREE_VIEW(me->tblData), colLogChan);

        GtkCellRenderer *playbackChanRenderer = gtk_cell_renderer_text_new();
        g_object_set(playbackChanRenderer, "editable", TRUE, (void*)NULL);
        g_signal_connect(playbackChanRenderer, "edited", G_CALLBACK(playbackChanEdit), me);
        GtkTreeViewColumn *colPlaybackChan =
            gtk_tree_view_column_new_with_attributes("Playback Channel",
                                                     playbackChanRenderer, "text",
                                                     PLAY_CHAN_COLUMN, (void*)NULL);
        gtk_tree_view_column_set_resizable(colPlaybackChan, TRUE);
        gtk_tree_view_column_set_expand(colPlaybackChan, TRUE);
        gtk_tree_view_append_column(GTK_TREE_VIEW(me->tblData), colPlaybackChan);

        GtkCellRenderer *enableRenderer = gtk_cell_renderer_toggle_new();
        GtkTreeViewColumn *colEnable =
            gtk_tree_view_column_new_with_attributes("Enable", enableRenderer,
                                                     "active", ENABLED_COLUMN, (void*)NULL);
        gtk_tree_view_column_set_resizable(colEnable, TRUE);
        gtk_tree_view_column_set_expand(colPlaybackChan, TRUE);
        g_signal_connect(enableRenderer, "toggled", G_CALLBACK(channelEnable), me);
        gtk_tree_view_append_column(GTK_TREE_VIEW(me->tblData), colEnable);

        GtkWidget *scrolled_window = gtk_scrolled_window_new(NULL, NULL);
        gtk_container_add(GTK_CONTAINER(scrolled_window), me->tblData);
        gtk_grid_attach(GTK_GRID(grid), scrolled_window, 0, 4, 6, 1);

        me->btnToggle = gtk_button_new_with_label("Toggle Selected");
        g_signal_connect(me->btnToggle, "clicked", G_CALLBACK(toggle), me);
        gtk_widget_set_hexpand(me->btnToggle, TRUE);
        gtk_widget_set_halign(me->btnToggle, GTK_ALIGN_FILL);
        gtk_grid_attach(GTK_GRID(grid), me->btnToggle, 0, 5, 1, 1);

        GtkWidget *lblPrefix = gtk_label_new("Channel Prefix:");
        gtk_widget_set_hexpand(lblPrefix, TRUE);
        gtk_widget_set_halign(lblPrefix, GTK_ALIGN_CENTER);
        gtk_grid_attach(GTK_GRID(grid), lblPrefix, 1, 5, 1, 1);

        me->txtPrefix = gtk_entry_new();
        gtk_widget_set_hexpand(me->txtPrefix, TRUE);
        gtk_widget_set_halign(me->txtPrefix, GTK_ALIGN_CENTER);
        g_signal_connect(me->txtPrefix, "changed", G_CALLBACK(prefixChanged), me);
        gtk_grid_attach(GTK_GRID(grid), me->txtPrefix, 2, 5, 4, 1);

        if (me->zcmIn && me->zcmIn->good()) {
            me->enableUI(true);
            for (size_t i = 0; i < me->bookmarks.size(); ++i) {
                auto &b = me->bookmarks[i];
                gtk_scale_add_mark(GTK_SCALE(me->sclMacroScrub),
                                   b.logTimePerc, GTK_POS_TOP,
                                   to_string(i).c_str());
                b.addedToGui = true;
            }
        } else {
            me->enableUI(false);
        }

        gtk_widget_show_all(me->window);
    }

    void quit()
    {
        unique_lock<mutex> lk(windowLk);
        if (window) gtk_window_close(GTK_WINDOW(window));
    }

    void redraw()
    {
        {
            unique_lock<mutex> lk(redrawLk);
            redrawNow = true;
        }
        redrawCv.notify_all();
    }

    void redrawThrFunc()
    {
        timespec delay;
        delay.tv_sec = 0;
        delay.tv_nsec = 3e7;
        while (true) {
            {
                unique_lock<mutex> lk(redrawLk);
                redrawCv.wait(lk, [&](){ return done || redrawNow; });
                if (done) return;
                redrawNow = false;
            }
            g_idle_add((GSourceFunc)zcmUpdateGui, this);
            nanosleep(&delay, nullptr);
        }
    }

    void playThrFunc()
    {
        thread thr(&LogPlayer::redrawThrFunc, this);

        // Wait for zcm log to become available
        while (true) {
            unique_lock<mutex> lk(zcmLk);
            zcmCv.wait(lk, [&](){
                return (zcmIn && zcmIn->good()) || done;
            });
            if (done) {
                quit();
                thr.join();
                return;
            }
            if (zcmIn && zcmIn->good()) break;
        }

        zcmIn->seekToTimestamp(0);

        uint64_t lastDispatchUtime = numeric_limits<uint64_t>::max();;;
        uint64_t lastLogUtime = numeric_limits<uint64_t>::max();;
        float lastSpeedTarget = numeric_limits<float>::max();
        zcm::Filter currSpeed(zcm::Filter::convergenceTimeToNatFreq(0.5, 0.8), 0.8);

        const zcm::LogEvent *le;

        redraw();

        uint64_t lastLoopUtime = numeric_limits<uint64_t>::max();

        while (true) {

            bool _isPlaying;
            float _speedTarget;
            bool wasPaused = false;
            uint64_t _requestTimeUs;
            map<string, ChannelInfo> _channelMap;
            {
                unique_lock<mutex> lk(zcmLk);
                zcmCv.wait(lk, [&](){
                    bool wakeup = done ||
                                  isPlaying ||
                                  requestTimeUs != numeric_limits<uint64_t>::max();
                    wasPaused |= !wakeup;
                    return wakeup;
                });
                if (done) break;
                if (!(isPlaying || requestTimeUs != numeric_limits<uint64_t>::max())) continue;

                _isPlaying = isPlaying;

                _speedTarget = speedTarget;

                _requestTimeUs = requestTimeUs;
                requestTimeUs = numeric_limits<uint64_t>::max();

                _channelMap = channelMap;
            }

            bool reset = _speedTarget != lastSpeedTarget || wasPaused;

            if (_requestTimeUs != numeric_limits<uint64_t>::max()) {
                zcmIn->seekToTimestamp(_requestTimeUs);
                reset = true;
            }

            le = zcmIn->readNextEvent();
            if (!le) {
                if (args.exitWhenDone) break;
                unique_lock<mutex> lk(zcmLk);
                isPlaying = false;
                redraw();
                continue;
            }

            uint64_t nowUs = TimeUtil::utime();
            if (lastLoopUtime == numeric_limits<uint64_t>::max()) lastLoopUtime = nowUs;

            if (reset) {
                lastLogUtime = le->timestamp;
                lastDispatchUtime = nowUs;
                lastSpeedTarget = _speedTarget;
                currSpeed.reset();
            }

            if (!_isPlaying) {
                unique_lock<mutex> lk(zcmLk);
                currMsgUtime = (uint64_t) le->timestamp;
                redraw();
                continue;
            }

            uint64_t localDiffUs = nowUs - lastDispatchUtime;

            // Total difference of timestamps of the current and
            // first message (after speed or play/pause changes)
            uint64_t logDiffUs = 0;
            if ((uint64_t)le->timestamp < lastLogUtime) {
                cerr << "Subsequent log events have utimes that are out of order" << endl
                     << "Consider running:" << endl
                     << "zcm-log-repair " << args.filename
                     << " -o " << args.filename << ".repaired" << endl;
            } else {
                logDiffUs = (uint64_t) le->timestamp - lastLogUtime;
            }

            // How much real time should have elapsed given the speed target
            uint64_t logDiffSpeedUs = logDiffUs / _speedTarget;

            uint64_t diffUs = logDiffSpeedUs > localDiffUs ? logDiffSpeedUs - localDiffUs : 0;

            // If we're early to publish the next message, then we're
            // achieving _speedTarget. If we're late, then how much we're
            // late by determines how far behind we are on speed
            double speed = logDiffSpeedUs >= localDiffUs ?
                           _speedTarget :
                           (float) logDiffSpeedUs / (float) localDiffUs * _speedTarget;

            double obsDt = (nowUs - lastLoopUtime) / 1e6;
            currSpeed.newObs(speed, obsDt);

            // Ensure nanosleep wakes up before the range of uncertainty of
            // the OS scheduler would impact our sleep time. Then we busy wait
            const uint64_t busyWaitUs = args.highAccuracyMode ? 10000 : 0;
            diffUs = diffUs > busyWaitUs ? diffUs - busyWaitUs : 0;

            // Sleep until we're supposed to wake up and busy wait
            if (diffUs != 0) {
                // Introducing time differences to starting times rather than last loop
                // times eliminates linear increase of delay when message are published
                timespec delay;
                delay.tv_sec = (long int) diffUs / 1000000;
                delay.tv_nsec = (long int) (diffUs - (delay.tv_sec * 1000000)) * 1000;
                nanosleep(&delay, nullptr);
            }
            // Busy wait the rest
            while (logDiffSpeedUs > TimeUtil::utime() - lastDispatchUtime);

            if (args.verbose)
                printf("%.3f Channel %-20s size %d\n", le->timestamp / 1e6,
                       le->channel.c_str(), le->datalen);

            ChannelInfo c;

            bool addToMainChannelMap = false;

            if (!_channelMap.count(le->channel)) {
                c.pubChannel = le->channel;
                c.enabled = true;
                c.addedToGui = false;
                _channelMap[le->channel] = c;
                addToMainChannelMap = true;
            }

            c = _channelMap[le->channel];

            if (c.enabled) zcmOut->publish(c.pubChannel.c_str(), le->data, le->datalen);

            {
                unique_lock<mutex> lk(zcmLk);

                if (addToMainChannelMap) channelMap[le->channel] = c;

                currMsgUtime = (uint64_t) le->timestamp;

                this->currSpeed = currSpeed[zcm::Filter::LOW_PASS];

                if (stepRequest) {
                    if (stepPrefix.empty()) {
                        isPlaying = false;
                        stepRequest = false;
                    } else if (le->channel.rfind(stepPrefix, 0) == 0) {
                        isPlaying = false;
                        stepRequest = false;
                    }
                }
            }

            redraw();

            lastLoopUtime = nowUs;
        }

        quit();
        thr.join();
    }

    int run()
    {
        GtkApplication *app = gtk_application_new("org.zcm.logplayer", G_APPLICATION_NON_UNIQUE);

        thread thr(&LogPlayer::playThrFunc, this);

        g_signal_connect(app, "activate", G_CALLBACK(activate), this);
        int ret = g_application_run(G_APPLICATION(app), 0, NULL);

        done = 1;
        wakeup();

        g_object_unref(app);

        thr.join();

        return ret;
    }
};

static LogPlayer lp;

static void sighandler(int signal)
{
    done++;
    if (done == 3) exit(1);
    lp.wakeup();
}

int main(int argc, char *argv[])
{
    if (!lp.init(argc, argv)) return 1;

    // Register signal handlers
    signal(SIGINT, sighandler);
    signal(SIGQUIT, sighandler);
    signal(SIGTERM, sighandler);

    return lp.run();
}
