#include <fstream>
#include <getopt.h>
#include <iostream>
#include <limits>
#include <memory>
#include <regex>
#include <vector>

#include <zcm/zcm-cpp.hpp>

#include "util/StringUtil.hpp"
#include "util/TypeDb.hpp"

using namespace std;

struct Args
{
    string inlog  = "";
    string outlog = "";
    bool debug    = false;
    bool verbose  = false;

    unique_ptr<TypeDb> types;

    enum ConditionSourceType
    {
        Seconds,
        Field,
        NumSourceTypes,
    };

    class FieldAccessor
    {
      public:
        const TypeMetadata* md = nullptr;
        int fIdx = -1;
        zcm_field_t field;

        FieldAccessor(const string& msgtype, const string& field, const TypeDb* types)
        {
            md = types->getByName(StringUtil::dotsToUnderscores(msgtype));
            if (!md) return;

            auto info = md->info;

            uint8_t *buf = new uint8_t[info->struct_size()];

            size_t numFields = (size_t)info->num_fields();
            for (size_t i = 0; i < numFields; ++i) {
                zcm_field_t f;
                assert(info->get_field(buf, i, &f) >= 0);

                if (string(f.name) == field) {
                    fIdx = i;
                    this->field = f;
                    break;
                }
            }

            delete[] buf;
        }

        bool good() const
        {
            return md && fIdx >= 0;
        }

        bool canBeBoolean() const
        {
            switch (field.type) {
                case ZCM_FIELD_INT8_T:
                case ZCM_FIELD_INT16_T:
                case ZCM_FIELD_INT32_T:
                case ZCM_FIELD_INT64_T:
                case ZCM_FIELD_BOOLEAN:
                    return true;
                default:
                    return false;
            }
        }

        bool canBeNumeric() const
        {
            switch (field.type) {
                case ZCM_FIELD_INT8_T:
                case ZCM_FIELD_INT16_T:
                case ZCM_FIELD_INT32_T:
                case ZCM_FIELD_INT64_T:
                case ZCM_FIELD_FLOAT:
                case ZCM_FIELD_DOUBLE:
                    return true;
                default:
                    return false;
            }
        }

        // Note: data must be a pointer to a struct of type msgtype (from constructor)
        bool get(const void* data, zcm_field_t& f) const
        {
            return md->info->get_field(data, fIdx, &f) >= 0;
        }
    };

    class Condition
    {
        ConditionSourceType type = ConditionSourceType::NumSourceTypes;

        // Valid only for type == ConditionSourceType::Field
        string channel;
        string msgtype;
        string msgfield;
        vector<FieldAccessor> accessors;

        bool compBool;

        bool inverted;

        double number;
        bool lessThan;

        bool isConfigured = false;

        bool boolIsActive(bool val) const
        {
            return inverted ? !val : val;
        }

        bool numberIsActive(double val) const
        {
            return lessThan ? val < number : val >= number;
        }

        struct FlaggedBool
        {
            bool isValid;
            bool val;
        };

        struct FlaggedNumber
        {
            bool isValid;
            double val;
        };

        bool getField(const zcm::LogEvent* le, zcm_field_t& f) const
        {
            assert(!accessors.empty());
            const auto* info = accessors.front().md->info;
            uint8_t* buf = new uint8_t[(size_t)info->struct_size()];
            if (info->decode(le->data, 0, le->datalen, buf) != le->datalen) {
                cerr << "Unable to decode " << accessors.front().md->name
                     << " on " << le->channel << endl;
                delete[] buf;
                return false;
            }
            void* lastData = (void*)buf;
            for (auto& a : accessors) {
                if (!a.get(lastData, f)) {
                    delete[] buf;
                    return false;
                }
                lastData = f.data;
            }
            delete[] buf;
            return true;
        }

        FlaggedBool extractBool(const zcm::LogEvent* le) const
        {
            if (le->channel != channel) return { false, false };

            zcm_field_t f;
            if (!getField(le, f)) return { false, false };
            bool ret;
            switch (f.type) {
                case ZCM_FIELD_INT8_T:  ret = *((int8_t*)f.data); break;
                case ZCM_FIELD_INT16_T: ret = *((int16_t*)f.data); break;
                case ZCM_FIELD_INT32_T: ret = *((int32_t*)f.data); break;
                case ZCM_FIELD_INT64_T: ret = *((int64_t*)f.data); break;
                case ZCM_FIELD_BOOLEAN: ret = *((bool*)f.data); break;
                default: return { false, false };
            }
            return { true, ret };
        }

        FlaggedNumber extractNumber(const zcm::LogEvent* le) const
        {
            if (le->channel != channel) return { false, 0.0 };

            zcm_field_t f;
            if (!getField(le, f)) return { false, 0.0 };
            double ret;
            switch (f.type) {
                case ZCM_FIELD_INT8_T:  ret = (double)*((int8_t*)f.data);
                case ZCM_FIELD_INT16_T: ret = (double)*((int16_t*)f.data);
                case ZCM_FIELD_INT32_T: ret = (double)*((int32_t*)f.data);
                case ZCM_FIELD_INT64_T: ret = (double)*((int64_t*)f.data);
                case ZCM_FIELD_FLOAT:   ret = (double)*((float*)f.data);
                case ZCM_FIELD_DOUBLE:  ret = *((double*)f.data);
                default:                return { false, 0.0 };
            }
            return { true, ret };
        }

      public:
        Condition() {}

        virtual ~Condition() {}

        virtual bool isFullySpecified() const { return isConfigured; }

        virtual bool setSeconds()
        {
            if (type != ConditionSourceType::NumSourceTypes) return false;
            type = ConditionSourceType::Seconds;
            return true;
        }

        virtual bool setField(const string& field, const TypeDb* types)
        {
            if (type != ConditionSourceType::NumSourceTypes) return false;
            type = ConditionSourceType::Field;

            auto parts = StringUtil::split(field, ':');
            if (parts.size() != 3) return false;

            channel = parts[0];
            msgtype = parts[1];
            msgfield = parts[2];
            auto fieldParts = StringUtil::split(msgfield, '.');

            for (size_t i = 0; i < fieldParts.size(); ++i) {
                accessors.emplace_back(i == 0 ?
                                       msgtype :
                                       string(accessors.back().field.typestr),
                                       fieldParts[i], types);
                if (!accessors.back().good()) return false;
            }

            return true;
        }

        virtual bool setCompound(bool _and) { return false; }

        virtual bool setCompBool(bool _inverted)
        {
            if (type == ConditionSourceType::NumSourceTypes) return false;
            if (type == ConditionSourceType::Seconds) return false;
            assert(!accessors.empty());
            if (!accessors.back().canBeBoolean()) return false;
            compBool = true;
            inverted = _inverted;
            isConfigured = true;
            return true;
        }

        virtual bool setCompNumber(double _number, bool _lessThan)
        {
            if (type == ConditionSourceType::NumSourceTypes) return false;
            if (type == ConditionSourceType::Field) {
                assert(!accessors.empty());
                if (!accessors.back().canBeBoolean()) return false;
            }
            number = _number;
            compBool = false;
            lessThan = _lessThan;
            isConfigured = true;
            return true;
        }

        virtual bool isActive(const zcm::LogEvent* le) const
        {
            assert(isFullySpecified());

            uint64_t leTimestamp = (uint64_t)le->timestamp;

            // Note this would need to move to a class variable if we wanted
            // to be able to process more than one log in one call of this
            // program. No other tools in zcm have that functionality so
            // ignoring that for now.
            static uint64_t firstUtime = numeric_limits<uint64_t>::max();
            if (leTimestamp < firstUtime) firstUtime = leTimestamp;

            switch (type) {
                case ConditionSourceType::Seconds: {
                    return numberIsActive((double)(leTimestamp - firstUtime) / 1e6);
                }
                case ConditionSourceType::Field: {
                    if (compBool) {
                        auto v = extractBool(le);
                        if (!v.isValid) return false;
                        return boolIsActive(v.val);
                    } else {
                        auto v = extractNumber(le);
                        if (!v.isValid) return false;
                        return numberIsActive(v.val);
                    }
                }
                case ConditionSourceType::NumSourceTypes: {
                    assert(false && "Should not be possible");
                    break;
                }
            }

            assert(false && "Unreachable");
            return false;
        }

        virtual void dump(size_t hangingIndent) const
        {
            string idt = string(hangingIndent, ' ');

            cout << idt << "Is configured: " << (isConfigured ? "true" : "false") << endl;

            cout << idt << "Type: ";
            switch (type) {
                case ConditionSourceType::Seconds:
                    cout << "Seconds" << endl;
                    break;
                case ConditionSourceType::Field:
                    cout << "Field" << endl;
                    cout << idt << "Channel: " << channel << endl;
                    cout << idt << "Msgtype: " << msgtype << endl;
                    cout << idt << "Msgfield: " << msgfield << endl;
                    break;
                case ConditionSourceType::NumSourceTypes: cout << "Unconfigured"; break;
            }

            cout << idt << "Trigger: " << (compBool ? "Boolean " : "Numeric ");

            if (compBool) {
                cout << (inverted ? "== false" : "== true") << endl;
            } else {
                cout << (lessThan ? "< " : ">= ") << number << endl;
            }

        }
    };

    class CompoundCondition : public Condition
    {
        // Note that this could eventually turn into more complex compound conditions
        bool _and; // false implies this is an "or" condition

      public:
        unique_ptr<Condition> cond1;
        unique_ptr<Condition> cond2;

        CompoundCondition(bool _and) : _and(_and) {}
        virtual ~CompoundCondition() {}

        bool isFullySpecified() const override
        {
            return cond1 && cond1->isFullySpecified() &&
                   cond2 && cond2->isFullySpecified();
        }

        bool setSeconds() override
        {
            if (!cond1) cond1.reset(new Condition());
            if (!cond1->isFullySpecified()) return cond1->setSeconds();
            if (!cond2) cond2.reset(new Condition());
            return cond2->setSeconds();
        }

        bool setField(const string& field, const TypeDb* types) override
        {
            if (!cond1) cond1.reset(new Condition());
            if (!cond1->isFullySpecified()) return cond1->setField(field, types);
            if (!cond2) cond2.reset(new Condition());
            return cond2->setField(field, types);
        }

        bool setCompound(bool _and) override
        {
            if (!cond1) cond1.reset(new CompoundCondition(_and));
            if (!cond1->isFullySpecified()) return false;
            if (!cond2) cond2.reset(new CompoundCondition(_and));
            return false;
        }

        bool setCompBool(bool inverted) override
        {
            if (!cond1) cond1.reset(new Condition());
            if (!cond1->isFullySpecified()) return cond1->setCompBool(inverted);
            if (!cond2) cond2.reset(new Condition());
            return cond2->setCompBool(inverted);
        }

        bool setCompNumber(double number, bool lessThan) override
        {
            if (!cond1) cond1.reset(new Condition());
            if (!cond1->isFullySpecified()) return cond1->setCompNumber(number, lessThan);
            if (!cond2) cond2.reset(new Condition());
            return cond2->setCompNumber(number, lessThan);
        }

        bool isActive(const zcm::LogEvent* le) const override
        {
            bool c1 = cond1->isActive(le);
            bool c2 = cond2->isActive(le);
            return _and ? c1 && c2 : c1 || c2;
        }

        void dump(size_t hangingIndent) const override
        {
            string idt = string(hangingIndent, ' ');
            string nidt = string(hangingIndent + 2, ' ');
            size_t nextHangingIndent = hangingIndent + 4;

            cout << idt << (_and ? "And:" : "Or:") << endl;
            cout << nidt << "Cond1: ";
            if (cond1) {
                cout << endl;
                cond1->dump(nextHangingIndent);
            } else {
                cout << "\033[31m"
                     << "unspecified"
                     << "\033[0m" << endl;
            }
            cout << nidt << "Cond2: ";
            if (cond2) {
                cout << endl;
                cond2->dump(nextHangingIndent);
            } else {
                cout << "\033[31m"
                     << "unspecified"
                     << "\033[0m" << endl;
            }
        }
    };

    class Region
    {
      public:
        unique_ptr<Condition> begin;
        unique_ptr<Condition> end;

        mutable bool firstTimeThrough = true;
        mutable bool active = false;

        vector<pair<string, regex>> channels;

        int64_t beginAdjustment = 0;
        int64_t endAdjustment = 0;

        int64_t getBeginAdjustment() const
        {
            return beginAdjustment;
        }

        int64_t getEndAdjustment() const
        {
            return endAdjustment;
        }

        bool isActive(const zcm::LogEvent* le) const
        {
            bool b = !begin || begin->isActive(le);
            bool e = !end || end->isActive(le);

            if (!begin && firstTimeThrough) active = true;
            firstTimeThrough = false;

            if (begin && b) active = true;
            if (end && e) active = false;
            return active;
        }

        bool keepEvent(const string& channel) const
        {
            if (channels.empty()) return true;
            for (auto& c : channels) {
                if (regex_match(channel, c.second)) {
                    return true;
                }
            }
            return false;
        }

        bool isFullySpecified() const
        {
            return (!begin || begin->isFullySpecified()) &&
                   (!end || end->isFullySpecified());
        }

        void dump(size_t hangingIndent) const
        {
            string idt = string(hangingIndent, ' ');
            size_t nextHangingIndent = hangingIndent + 2;
            string nidt = string(nextHangingIndent, ' ');

            cout << idt << "Channels: ";
            if (channels.empty()) cout << "All";
            cout << endl;
            cout << idt << "Begin Adjustment: " << (beginAdjustment / 1e6) << "s" << endl;
            cout << idt << "End Adjustment: " << (endAdjustment / 1e6) << "s" << endl;
            for (auto& c : channels) cout << nidt << c.first << endl;
            cout << idt << "Begin: ";
            if (begin) {
                cout << endl;
                begin->dump(nextHangingIndent);
            } else {
                cout << "beginning of log" << endl;
            }
            cout << idt << "End: ";
            if (end) {
                cout << endl;
                end->dump(nextHangingIndent);
            } else {
                cout << "end of log" << endl;
            }
        }
    };

    class RegionsFactory
    {
        bool specifyingEnd = true;

      public:
        vector<Region> regions;

        RegionsFactory() {}

        bool addBegin()
        {
            if (!regions.empty() && !regions.back().isFullySpecified()) return false;
            regions.emplace_back();
            specifyingEnd = false;
            return true;
        }

        bool addEnd()
        {
            if (!regions.empty() && !regions.back().isFullySpecified()) return false;
            if (regions.empty()) regions.emplace_back();
            auto& back = regions.back();
            if (back.end) {
                if (!back.end->isFullySpecified()) return false;
                regions.emplace_back();
            }

            specifyingEnd = true;
            return true;
        }

        bool addCompound(bool _and)
        {
            if (regions.empty()) return false;

            auto& back = regions.back();

            if (!specifyingEnd) {
                if (!back.begin) {
                    back.begin.reset(new CompoundCondition(_and));
                } else {
                    back.begin->setCompound(_and);
                }
            } else {
                if (!back.end) {
                    back.end.reset(new CompoundCondition(_and));
                } else {
                    back.end->setCompound(_and);
                }
            }

            return true;
        }

        bool setConditionAsTime()
        {
            if (regions.empty()) return false;

            auto& back = regions.back();

            if (!specifyingEnd) {
                if (!back.begin) back.begin.reset(new Condition());
                if (!back.begin->setSeconds()) return false;
            } else {
                if (!back.end) back.end.reset(new Condition());
                if (!back.end->setSeconds()) return false;
            }

            return true;
        }

        bool setConditionAsField(const string& field, const TypeDb* types)
        {
            if (regions.empty()) return false;

            auto& back = regions.back();

            if (!specifyingEnd) {
                if (!back.begin) back.begin.reset(new Condition());
                if (!back.begin->setField(field, types)) return false;
            } else {
                if (!back.end) back.end.reset(new Condition());
                if (!back.end->setField(field, types)) return false;
            }

            return true;
        }

        bool setConditionCompBool(bool inverted)
        {
            if (regions.empty()) return false;

            auto& back = regions.back();

            if (!specifyingEnd) {
                if (!back.begin) back.begin.reset(new Condition());
                if (!back.begin->setCompBool(inverted)) return false;
            } else {
                if (!back.end) back.end.reset(new Condition());
                if (!back.end->setCompBool(inverted)) return false;
            }

            return true;
        }

        bool setConditionCompNumber(double number, bool lessThan)
        {
            if (regions.empty()) return false;

            auto& back = regions.back();

            if (!specifyingEnd) {
                if (!back.begin) back.begin.reset(new Condition());
                if (!back.begin->setCompNumber(number, lessThan)) return false;
            } else {
                if (!back.end) back.end.reset(new Condition());
                if (!back.end->setCompNumber(number, lessThan)) return false;
            }

            return true;
        }

        bool setChannels(const string& _channels)
        {
            if (regions.empty()) return false;
            auto& back = regions.back();
            auto chans = StringUtil::split(_channels, ',');
            for (const auto& c : chans) back.channels.emplace_back(c, regex(c));
            return true;
        }

        bool setBeginAdjustment(double beginAdjustmentSeconds)
        {
            if (regions.empty()) return false;
            auto& back = regions.back();
            back.beginAdjustment = beginAdjustmentSeconds * 1e6;
            return true;
        }

        bool setEndAdjustment(double endAdjustmentSeconds)
        {
            if (regions.empty()) return false;
            auto& back = regions.back();
            back.endAdjustment = endAdjustmentSeconds * 1e6;
            return true;
        }

        void dump()
        {
            cout << "Regions: " << endl;
            for (size_t i = 0; i < regions.size(); ++i) {
                cout << "  Region " << i << endl;
                regions[i].dump(4);
            }
        }
    };
    RegionsFactory factory;

    bool parse(int argc, char *argv[])
    {
        // set some defaults
        const char *optstring = "i:o:t:dvbec:arsf:n:l:g:h";
        struct option long_opts[] = {
            { "input",     required_argument, 0, 'i' },
            { "output",    required_argument, 0, 'o' },
            { "type-path", required_argument, 0, 't' },
            { "debug",     no_argument,       0, 'd' },
            { "verbose",   no_argument,       0, 'v' },
            { "help",      no_argument,       0, 'h' },

            { "begin", no_argument, 0, 'b' },
            { "end",   no_argument, 0, 'e' },

            { "channels",     required_argument, 0, 'c' },
            { "adjust-begin", required_argument, 0,  1  },
            { "adjust-end",   required_argument, 0,  2  },

            { "and", no_argument, 0, 'a' },
            { "or",  no_argument, 0, 'r' },

            { "seconds",       no_argument, 0, 's' },
            { "field",   required_argument, 0, 'f' },

            { "boolean",      required_argument, 0, 'n' },
            { "less-than",    required_argument, 0, 'l' },
            { "greater-than", required_argument, 0, 'g' },

            { 0, 0, 0, 0 }
        };

        int c;
        while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0) {
            switch (c) {
                case 'i': inlog  = string(optarg); break;
                case 'o': outlog = string(optarg); break;
                case 't': {
                    types.reset(new TypeDb(string(optarg), debug));
                    break;
                }
                case 'd': debug = true; break;
                case 'v': verbose = true; break;

                case 'b': {
                    if (!factory.addBegin()) {
                        cerr << "Improperly placed begin condition" << endl;
                        return false;
                    }
                    break;
                }
                case 'e': {
                    if (!factory.addEnd()) {
                        cerr << "Improperly placed end condition" << endl;
                        return false;
                    }
                    break;
                }
                case 'c': {
                    if (!factory.setChannels(optarg)) {
                        cerr << "Improperly placed channels" << endl;
                        return false;
                    }
                    break;
                }
                case 1: {
                    if (!factory.setBeginAdjustment(atof(optarg))) {
                        cerr << "Improperly placed begin adjustment" << endl;
                        return false;
                    }
                    break;
                }
                case 2: {
                    if (!factory.setEndAdjustment(atof(optarg))) {
                        cerr << "Improperly placed end adjustment" << endl;
                        return false;
                    }
                    break;
                }

                case 'a': {
                    if (!factory.addCompound(true)) {
                        cerr << "Improperly placed and condition" << endl;
                        return false;
                    }
                    break;
                }
                case 'r': {
                    if (!factory.addCompound(false)) {
                        cerr << "Improperly placed or condition" << endl;
                        return false;
                    }
                    break;
                }

                case 's': {
                    if (!factory.setConditionAsTime()) {
                        cerr << "Improperly specified time condition" << endl;
                        return false;
                    }
                    break;
                }
                case 'f': {
                    if (!types) {
                        cerr << "Must specify types.so before field conditions" << endl;
                        return false;
                    }
                    if (!factory.setConditionAsField(optarg, types.get())) {
                        cerr << "Improperly specified field condition" << endl;
                        return false;
                    }
                    break;
                }
                case 'n': {
                    bool invert;
                    if (string(optarg) == "inverted") {
                        invert = true;
                    } else if (string(optarg) == "normal") {
                        invert = false;
                    } else {
                        cerr << "Invalid boolean specifier: " << optarg << endl;
                        return false;
                    }
                    if (!factory.setConditionCompBool(invert)) {
                        cerr << "Improperly specified bool comparison" << endl;
                        return false;
                    }
                    break;
                }
                case 'l': {
                    if (!factory.setConditionCompNumber(atof(optarg), true)) {
                        cerr << "Improperly specified less than comparison" << endl;
                        return false;
                    }
                    break;
                }
                case 'g': {
                    if (!factory.setConditionCompNumber(atof(optarg), false)) {
                        cerr << "Improperly specified greater equal comparison" << endl;
                        return false;
                    }
                    break;
                }

                case 'h':
                default: usage(); return false;
            };
        }

        if (inlog == "") {
            cerr << "Please specify logfile input" << endl;
            return false;
        }

        if (outlog == "") {
            cerr << "Please specify log file output" << endl;
            return false;
        }

        if (factory.regions.empty()) {
            cerr << "Please specify at least one region" << endl;
            return false;
        }

        for (size_t i = 0; i < factory.regions.size(); ++i) {
            if (!factory.regions[i].isFullySpecified()) {
                cerr << "Region " << i << " is not fully specified" << endl;
                return false;
            }
        }

        return true;
    }

    void usage()
    {
        cout << "usage: zcm-log-filter [options]" << endl
             << "" << endl
             << "    Filter a log to events in regions marked by begin and end conditions" << endl
             << endl
             << "Examples:" << endl
             << "    # Filter a log to all events between 10 and 20 seconds after log start" << endl
             << "    zcm-log-filter -i in.log -o out.log -b -s -g 10 -e -s -g 20" << endl
             << endl
             << "    # Filter a log to all events after a boolean flag goes true" << endl
             << "    zcm-log-filter -i in.log -o out.log -t types.so "
                                   "-b -f FLAGS:flags_t:field1 -n normal" << endl
             << endl
             << "Options:" << endl
             << endl
             << "  -h, --help              Shows this help text and exits" << endl
             << "  -i, --input=logfile     Input log to filter" << endl
             << "  -o, --output=logfile    Output filtered log file" << endl
             << "  -t, --type-path=path    Path to shared library containing zcmtypes" << endl
             << "  -d, --debug             Run a dry run to ensure proper setup" << endl
             << endl
             << "  Conditions" << endl
             << "    -b, --begin                  Interpret the next args as defining begin conditions" << endl
             << "    -e, --end                    Interpret the next args as defining end conditions" << endl
             << "    -c, --channels               Channels to keep when inside region" << endl
             << "                                 Provide as a comma separated list. Regex supported" << endl
             << "        --adjust-begin           Number of seconds to adjust the beginning of each" << endl
             << "                                 region by. Negative numbers indicate beginning" << endl
             << "                                 should be moved to an earlier time" << endl
             << "        --adjust-end             Number of seconds to adjust the end of each" << endl
             << "                                 region by. Negative numbers indicate end" << endl
             << "                                 should be moved to an earlier time" << endl
             << endl
             << "    -s, --seconds                Condition is based on seconds since beginning of zcm log" << endl
             << "    -f, --field=<field>          A config-style description of a flag in a channel/message" << endl
             << "                                 Formatted as: CHANNEL_NAME:MESSAGE_TYPE_LONG_NAME:FIELD" << endl
             << "                                 Where FIELD can be nested fields using \".\" as the" << endl
             << "                                 nested field accessor" << endl
             << endl
             << "    -n, --boolean=<inverted>     Evaluate condition as \"FIELD == true\"  for -n normal" << endl
             << "                                                    as \"FIELD == false\" for -n inverted" << endl
             << "                                 All other strings after -n are invalid" << endl
             << "    -l, --less-than=<number>     Evaluate condition as \"FIELD < number\"" << endl
             << "                                 Currently only supports comparing \"field\" to double" << endl
             << "    -g, --greater-than=<number>  Evaluate condition as \"FIELD >= number\"" << endl
             << "                                 Currently only supports comparing \"field\" to double" << endl
             << endl
             << "    Compounds:" << endl
             << "      -a, --and  And" << endl
             << "      -r, --or   Or" << endl
             << endl
             << "      For compound expressions, put the compound specifier before the two" << endl
             << "      expressions you want it to act on" << endl
             << endl
             << "      Example:" << endl
             << "        # To filter for all events starting after any event that" << endl
             << "        # occurs between 10 and 20s into the log and ending after" << endl
             << "        # any event 100s into the log. If there is no event in the" << endl
             << "        # log between 10 and 20 seconds after the beginning of the" << endl
             << "        # log, the region's begin condition will never trigger and" << endl
             << "        # the output log will be empty." << endl
             << "        zcm-log-filter -i in.log -o out.log -b -a -s -g 10 -s -l 20 -e -s -g 100" << endl
             << endl << endl;
    }
};

int main(int argc, char* argv[])
{
    Args args;
    bool success = args.parse(argc, argv);
    if (!args.factory.regions.empty()) {
        cout << endl;
        args.factory.dump();
        cout << endl;
    }
    if (!success) return 1;

    zcm::LogFile inlog(args.inlog, "r");
    if (!inlog.good()) {
        cerr << "Unable to open input zcm log: " << args.inlog << endl;
        return 1;
    }

    zcm::LogFile outlog(args.outlog, "w");
    if (!outlog.good()) {
        cerr << "Unable to open output zcm log: " << args.outlog << endl;
        return 1;
    }

    if (args.debug) return 0;

    auto processLog = [&inlog](function<void(const zcm::LogEvent* evt)> processEvent) {
        const zcm::LogEvent* evt;
        off64_t offset;
        static int lastPrintPercent = 0;

        fseeko(inlog.getFilePtr(), 0, SEEK_END);
        off64_t logSize = ftello(inlog.getFilePtr());
        fseeko(inlog.getFilePtr(), 0, SEEK_SET);

        while (1) {
            offset = ftello(inlog.getFilePtr());

            int percent = 100.0 * offset / (logSize == 0 ? 1 : logSize);
            if (percent != lastPrintPercent) {
                cout << "\r" << "Percent Complete: " << percent << flush;
                lastPrintPercent = percent;
            }

            evt = inlog.readNextEvent();
            if (evt == nullptr) break;

            processEvent(evt);
        }
        if (lastPrintPercent != 100) cout << "\r" << "Percent Complete: 100" << flush;
        cout << endl;
    };

    static constexpr int64_t i64max = numeric_limits<int64_t>::max();

    cout << "Marking regions on first pass..." << endl;
    vector<vector<pair<int64_t, int64_t>>> regionActiveZones;
    regionActiveZones.resize(args.factory.regions.size());
    processLog([&](const zcm::LogEvent* evt){
        for (size_t i = 0; i < regionActiveZones.size(); ++i) {
            auto& r = regionActiveZones[i];
            bool active = args.factory.regions[i].isActive(evt);
            if (!active) {
                if (!r.empty() && r.back().second != i64max)
                    r.emplace_back(i64max, i64max);
            } else {
                if (r.empty()) r.emplace_back(i64max, i64max);
                if (r.back().first == i64max) r.back().first = evt->timestamp;
                r.back().second = evt->timestamp;
            }
        }
    });

    size_t numRegions = 0;
    for (size_t i = 0; i < regionActiveZones.size(); ++i) {
        if (!regionActiveZones[i].empty() && regionActiveZones[i].back().first == i64max)
            regionActiveZones[i].pop_back();
        numRegions += regionActiveZones[i].size();
        if (args.debug) cout << "Region " << i << ":" << endl;
        for (size_t j = 0; j < regionActiveZones[i].size(); ++j) {
            auto& r = regionActiveZones[i][j];
            if (args.verbose) cout << "\t[" << r.first << ", " << r.second << "] -> ";
            r.first += args.factory.regions[i].getBeginAdjustment();
            r.second += args.factory.regions[i].getEndAdjustment();
            if (args.verbose) cout << "[" << r.first << ", " << r.second << "]" << endl;
        }
    }
    cout << "Adjusted endpoints of " << numRegions << " region"
         << (numRegions > 1 ? "s" : "") << "..." << endl;

    cout << "Writing events to output log..." << endl;
    size_t numInEvents = 0, numOutEvents = 0;
    processLog(
        [&args, &regionActiveZones, &inlog,
         &outlog, &numInEvents, &numOutEvents](const zcm::LogEvent* evt) {
        bool keepEvent = false;
        for (size_t i = 0; i < regionActiveZones.size(); ++i) {
            auto& fr = args.factory.regions[i];
            for (size_t j = 0; j < regionActiveZones[i].size(); ++j) {
                const auto& r = regionActiveZones[i][j];
                if (r.first <= evt->timestamp && evt->timestamp < r.second &&
                    fr.keepEvent(evt->channel)) {
                    keepEvent = true;
                    break;
                }
            }
            if (keepEvent) break;
        }

        if (keepEvent) {
            outlog.writeEvent(evt);
            numOutEvents++;
        }

        numInEvents++;
    });

    inlog.close();
    outlog.close();

    cout << "Filtered " << numInEvents << " events down to "
         << numOutEvents << " events" << endl;

    return 0;
}
