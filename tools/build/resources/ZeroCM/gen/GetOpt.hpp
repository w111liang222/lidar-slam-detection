#pragma once
#include "Common.hpp"

struct GetOpt
{
    struct Opt
    {
        string sname;
        string lname;
        string svalue;

        string help;
        int    type = 0;

        bool spacer = false;

        // did the user actually specify this option
        // (as opposed to a default value)?
        bool wasSpecified = false;
    };

    vector<Opt>                opts;
    unordered_map<string, int> lopts; // flag -> opts-index
    unordered_map<string, int> sopts; // flag -> opts-index
    vector<string>             extraargs;

    bool parse(int argc, char *argv[], int showErrors);
    void doUsage();

    void addSpacer(const string& s);
    void addBool(char sopt, const string& lname, bool def, const string& help);
    void addInt(char sopt, const string& lname, const string& def, const string& help);
    void addString(char sopt, const string& lname, const string& def, const string& help);


    Opt *findWithLOpt(const string& lname);
    Opt *findWithSOpt(const string& sname);

    const string& getString(const string& lname);
    int getInt(const string& lname);
    bool getBool(const string& lname);

    bool wasSpecified(const string& lname);
};
