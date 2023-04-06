#pragma once

#include "Common.hpp"
#include "GetOpt.hpp"

void setupOptionsC(GetOpt& gopt);
int emitC(const ZCMGen& zcm);
vector<string> getFilepathsC(const ZCMGen& zcm);
unordered_set<string> getReservedKeywordsC();

void setupOptionsCpp(GetOpt& gopt);
int emitCpp(const ZCMGen& zcm);
vector<string> getFilepathsCpp(const ZCMGen& zcm);
unordered_set<string> getReservedKeywordsCpp();

void setupOptionsJava(GetOpt& gopt);
int emitJava(const ZCMGen& zcm);
vector<string> getFilepathsJava(const ZCMGen& zcm);
unordered_set<string> getReservedKeywordsJava();

void setupOptionsPython(GetOpt& gopt);
int emitPython(const ZCMGen& zcm);
vector<string> getFilepathsPython(const ZCMGen& zcm);
unordered_set<string> getReservedKeywordsPython();

void setupOptionsNode(GetOpt& gopt);
int emitNode(const ZCMGen& zcm);
vector<string> getFilepathsNode(const ZCMGen& zcm);
unordered_set<string> getReservedKeywordsNode();

void setupOptionsJulia(GetOpt& gopt);
int emitJulia(const ZCMGen& zcm);
vector<string> getFilepathsJulia(const ZCMGen& zcm);
unordered_set<string> getReservedKeywordsJulia();
