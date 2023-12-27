#include "Emitter.hpp"

Emitter::Emitter(const string& fname)
{
    filename = fname;
    f = fopen(fname.c_str(), "w");
}

Emitter::~Emitter()
{
    if (f) fclose(f);
}

bool Emitter::good()
{
    return f != nullptr;
}

void Emitter::emit(int n, const char* msg, ...)
{
    va_list va;
    va_start(va, msg);

    fprintf(f, "%*s", indent(n), "");
    vfprintf(f, msg, va);
    fprintf(f, "\n");

    va_end(va);
}

void Emitter::emitStart(int n, const char* msg, ...)
{
    va_list va;
    va_start(va, msg);

    fprintf(f, "%*s", indent(n), "");
    vfprintf(f, msg, va);

    va_end(va);
}

void Emitter::emitContinue(const char* msg, ...)
{
    va_list va;
    va_start(va, msg);
    vfprintf(f, msg, va);
    va_end(va);
}

void Emitter::emitEnd(const char* msg, ...)
{
    va_list va;
    va_start(va, msg);
    vfprintf(f, msg, va);
    fprintf(f, "\n");
    va_end(va);
}
