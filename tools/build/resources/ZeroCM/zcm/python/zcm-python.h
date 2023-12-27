#include "Python.h"

void PyEval_InitThreads_CUSTOM()
{
#if PY_MAJOR_VERSION == 3 && PY_MINOR_VERSION >= 7
    Py_Initialize();
#else
    PyEval_InitThreads();
#endif
}
