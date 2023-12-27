#include "GetOpt.hpp"

static string EMPTY_STRING;

enum OptType {
    OPT_BOOL_TYPE = 1,
    OPT_STRING_TYPE = 2,
};

bool GetOpt::parse(int argc, char *argv[], int showErrors)
{
    bool okay = true;
    vector<string> toks;

    // take the input stream and chop it up into tokens
    for (int i = 1; i < argc; i++) {
        string arg = argv[i];
        char *eq = (char *)strstr(arg.c_str(), "=");

        // no equal sign? Push the whole thing.
        if (eq == NULL) {
            toks.push_back(std::move(arg));
        } else {
            // there was an equal sign. Push the part
            // before and after the equal sign
            char *val = &eq[1];
            eq[0] = 0;
            toks.push_back(arg.c_str());

            // if the part after the equal sign is
            // enclosed by quotation marks, strip them.
            if (val[0]=='\"') {
                int last = strlen(val) - 1;
                if (val[last]=='\"')
                    val[last] = 0;
                toks.push_back(&val[1]);
            } else {
                toks.push_back(val);
            }
        }
    }

    // now loop over the elements and evaluate the arguments
    size_t i = 0;
    while (i < toks.size()) {
        auto& tok_ = toks[i];
        char *tok = (char*)tok_.c_str();

        if (!strncmp(tok,"--", 2)) {
            char *optname = &tok[2];
            auto *opt = findWithLOpt(optname);
            if (!opt) {
                okay = false;
                if (showErrors)
                    printf("Unknown option --%s\n", optname);
                i++;
                continue;
            }

            opt->wasSpecified = true;

            if (opt->type == OPT_BOOL_TYPE) {
                if ((i+1) < toks.size()) {
                    auto& val = toks[i+1];
                    if (val == "true") {
                        i+=2;
                        opt->svalue = "true";
                        continue;
                    }
                    if (val == "false") {
                        i+=2;
                        opt->svalue = "false";
                        continue;
                    }
                }

                opt->svalue = "true";
                i++;
                continue;
            }

            if (opt->type == OPT_STRING_TYPE) {
                if ((i+1) < toks.size()) {
                    auto& val = toks[i+1];
                    i+=2;
                    opt->svalue = val;
                    continue;
                }

                okay = false;
                if (showErrors)
                    printf("Option %s requires a string argument.\n", optname);
            }
        }

        if (!strncmp(tok,"-",1) && strncmp(tok,"--",2)) {
            int len = strlen(tok);
            int pos;
            for (pos = 1; pos < len; pos++) {
                char sopt[2];
                sopt[0] = tok[pos];
                sopt[1] = 0;

                auto *opt = findWithSOpt(sopt);
                if (!opt) {
                    // is the argument a numerical literal that happens to be negative?
                    if (pos==1 && isdigit(tok[pos])) {
                        extraargs.push_back(tok);
                        break;
                    } else {
                        okay = false;
                        if (showErrors)
                            printf("Unknown option -%c\n", tok[pos]);
                        i++;
                        continue;
                    }
                }

                opt->wasSpecified = true;

                if (opt->type ==OPT_BOOL_TYPE) {
                    opt->svalue = "true";
                    continue;
                }

                if (opt->type == OPT_STRING_TYPE) {
                    if ((i+1) < toks.size()) {
                        auto& val = toks[i+1];
                        if (val[0]=='-')
                        {
                            okay = false;
                            if (showErrors)
                                printf("Ran out of arguments for option block %s\n", tok);
                        }
                        i++;

                        opt->svalue = val;
                        continue;
                    }

                    okay = false;
                    if (showErrors)
                        printf("Option -%c requires a string argument.\n", tok[pos]);
                }
            }
            i++;
            continue;
        }

        // it's not an option-- it's an argument.
        extraargs.push_back(tok);
        i++;
    }
    return okay;
}

void GetOpt::doUsage()
{
    int leftmargin = 2;
    int longwidth = 12;
    int valuewidth = 10;

    for (auto& opt : opts) {
        if (opt.spacer)
            continue;
        longwidth = std::max(longwidth, (int)opt.lname.size());
        if (opt.type == OPT_STRING_TYPE)
            valuewidth = std::max(valuewidth, (int)opt.svalue.size());
    }
    for (auto& opt : opts) {
        if (opt.spacer) {
            if (opt.help == "")
                printf("\n");
            else
                printf("\n%*s%s\n\n", leftmargin, "", opt.help.c_str());
            continue;
        }

        printf("%*s", leftmargin, "");

        if (opt.sname == "")
            printf("     ");
        else
            printf("-%c | ", opt.sname[0]);

        printf("--%*s ", -longwidth, opt.lname.c_str());

        printf(" [ %s ]", opt.svalue.c_str());

        printf("%*s", (int) (valuewidth - opt.svalue.size()), "");

        printf(" %s   ", opt.help.c_str());
        printf("\n");

    }
}

void GetOpt::addSpacer(const string& s)
{
    Opt o;
    o.spacer = true;
    o.help = s;
    opts.push_back(std::move(o));
}

void GetOpt::addBool(char sopt, const string& lname, bool def, const string& help)
{
    string sname = sopt ? string(1, sopt) : "";

    Opt o;
    o.sname = sopt ? string(1, sopt) : "";
    o.lname = lname;
    o.svalue = def ? "true" : "false";
    o.type = OPT_BOOL_TYPE;
    o.help = help;

    size_t idx = opts.size();
    opts.push_back(std::move(o));
    lopts[lname] = idx;
    sopts[sname] = idx;
}

void GetOpt::addInt(char sopt, const string& lname, const string& def, const string& help)
{
    addString(sopt, lname, def, help);
}

void GetOpt::addString(char sopt, const string& lname, const string& def, const string& help)
{
    string sname = sopt ? string(1, sopt) : "";

    Opt o;
    o.sname = sname;
    o.lname = lname;
    o.svalue = def;
    o.type = OPT_STRING_TYPE;
    o.help = help;

    size_t idx = opts.size();
    opts.push_back(std::move(o));
    lopts[lname] = idx;
    sopts[sname] = idx;
}


GetOpt::Opt *GetOpt::findWithLOpt(const string& lname)
{
    auto it = lopts.find(lname);
    if (it != lopts.end())
        return &opts[it->second];
    else
        return nullptr;
}

GetOpt::Opt *GetOpt::findWithSOpt(const string& sname)
{
    auto it = sopts.find(sname);
    if (it != sopts.end())
        return &opts[it->second];
    else
        return nullptr;
};


const string& GetOpt::getString(const string& lname)
{
    if (auto *opt = findWithLOpt(lname)) {
        return opt->svalue;
    } else {
        return EMPTY_STRING;
    }
}

int GetOpt::getInt(const string& lname)
{
    auto& v = getString(lname);
    assert(v != "");
    return atoi(v.c_str());
}

bool GetOpt::getBool(const string& lname)
{
    auto& v = getString(lname);
    assert(v != "");
    return (v == "true");
}

bool GetOpt::wasSpecified(const string& lname)
{
    if (auto *opt = findWithLOpt(lname)) {
        return opt->wasSpecified;
    } else {
        return false;
    }
}
