#pragma once
#include "Common.hpp"

struct Emitter
{
    Emitter(const string& fname);
    ~Emitter();
    bool good();

    static inline int indent(int n) { return 4*n; }

    void emit(int indent, const char* msg, ...);
    void emitStart(int indent, const char* msg, ...);
    void emitContinue(const char* msg, ...);
    void emitEnd(const char* msg, ...);

    const string& getFilename();

  private:
    FILE* f = nullptr;
    string filename = "";
};
