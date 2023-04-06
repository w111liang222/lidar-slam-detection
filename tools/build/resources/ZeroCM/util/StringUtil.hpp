#pragma once
#include <string>
using std::string;
#include <vector>
using std::vector;
#include <sstream>
#include <cassert>
#include <cstring>
#include <libgen.h>

namespace StringUtil
{
    // Splits string with delimiter into a vector
    // Example:
    //    split("fg/kjh/i", '/') -> vector<string>{"fg", "kjh", "i"}
    // This function is very useful for extracting delimited fields
    static vector<string> split(const string& str, char delimiter)
    {
        // TODO: if this function is ever too slow, it can be rewritten to loop through
        // a c-string and set '\0' at every delimiter, and then .emplace those char*
        // directly into the vector<>
        // Alternatively, a special object could be created to cut down on malloc()
        // Only a single malloc of the string should be needed and then simply slice
        // up the memory. It could even work as an iterator in a lazy style
        // Something like python generators... Ideas Ideas.
        // However, it's unlikely any of this will be needed, but a boy can dream right?

        vector<string> v;
        std::stringstream ss {str};
        string tok;

        while(getline(ss, tok, delimiter))
            v.push_back(std::move(tok));

        // NOTE: getline always discards the delimiter, so without the following code
        //      split("f/k", '/')  -> vector<string>{"f", "k"}
        //      split("f/k/", '/') -> vector<string>{"f", "k"}
        //   Our semantics require:
        //      split("f/k", '/')  -> vector<string>{"f", "k"}
        //      split("f/k/", '/') -> vector<string>{"f", "k", ""}
        // So, we check for the original string ending in the delimiter, and fix up the vector
        auto len = str.size();
        if (len > 0 && str[len-1] == delimiter)
            v.push_back("");

        return v;
    }

    static inline string join(const vector<string>& vec, const string& sep)
    {
        string ret;
        for (size_t i = 0; i < vec.size(); ++i) {
            ret += vec[i];
            if (i != vec.size()-1)
                ret += sep;
        }
        return ret;
    }

    static inline string join(const vector<string>& vec, char sep)
    {
        return join(vec, string(1, sep));
    }

    static inline string toUpper(const string& s)
    {
        string ret = s;
        for (auto& c : ret)
            c = toupper(c);
        return ret;
    }

    // Strings leading and trailing whitespace
    static inline string strip(const string& s)
    {
        size_t start = 0;
        for (; start < s.size(); ++start)
            if (s[start] != ' ' && s[start] != '\t')
                break;

        // If we've reached the end of the string, its all whitespace
        if (start == s.size())
            return "";

        size_t end = s.size();
        for (; end > 0; --end)
            if (s[end-1] != ' ' && s[end-1] != '\t')
                break;

        // We must have found a non-white
        assert(end != 0);

        // There should be no way that end <= start;
        assert(start < end);

        return string(s.c_str()+start, end-start);
    }

    static inline string replace(const string& s, char from, char to)
    {
        string ret = s;
        for (auto& c : ret)
            if (c == from) c = to;
        return ret;
    }

    static inline bool endswith(const string& s, const string& suffix)
    {
        if (s.size() < suffix.size())
            return false;
        const char *ending = &s[s.size()-suffix.size()];
        return 0 == ::strcmp(ending, suffix.c_str());
    }

    static inline string dotsToUnderscores(const string& s)
    { return replace(s, '.', '_'); }

    static inline string dotsToSlashes(const string& s)
    { return replace(s, '.', '/'); }

    static std::string basename(const std::string& path)
    {
        std::string cpy = path;
        return std::string(::basename((char*)cpy.c_str()));
    }
}
