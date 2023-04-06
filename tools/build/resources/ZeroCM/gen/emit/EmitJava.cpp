#include "Common.hpp"
#include "GetOpt.hpp"
#include "ZCMGen.hpp"
#include "Emitter.hpp"
#include "util/StringUtil.hpp"
#include "util/FileUtil.hpp"

#include <iostream>

static string dotsToSlashes(const string& s)
{
    return StringUtil::replace(s, '.', '/');
}

void setupOptionsJava(GetOpt& gopt)
{
    gopt.addString(0, "jpath",      "",        "Java file destination directory");
    gopt.addBool(0,   "jmkdir",     1,         "Make java source directories automatically");
    gopt.addString(0, "jdecl",      "implements zcm.zcm.ZCMEncodable", "String added to class declarations");
    gopt.addString(0, "jpkgprefix", "zcmtypes",
                      "Java package prefix, all types/packages will be inside this. "
                      "Comes *before* global pkg-prefix if both specified.");
}

struct PrimInfo
{
    string storage;
    string decode;
    string encode;

    PrimInfo(const string& storage, const string& decode, const string& encode) :
        storage(storage), decode(decode), encode(encode) {}
};

string makeFqn(const ZCMGen& zcm, const string& typeName)
{
    string ret = "";
    if (zcm.gopt->wasSpecified("jpkgprefix"))
        ret += zcm.gopt->getString("jpkgprefix") + ".";
    return ret + typeName;
}

// /** # -> replace1
//     @ -> replace2
//     $ -> replace3
// **/
static string specialReplace(const string& haystack,
                             const string& replace1,
                             const string& replace2 = "",
                             const string& replace3 = "")
{
    string ret;
    for (auto& c : haystack) {
        if (c == '#')
            ret += replace1;
        else if (c == '@')
            ret += replace2;
        else if (c == '$')
            ret += replace3;
        else
            ret += string(1, c);
    }
    return ret;
}

static string makeAccessor(const ZCMMember& zm, const string& obj)
{
    string ret;
    ret += obj + (obj.size() == 0 ? "" : ".") + zm.membername;
    for (size_t d = 0 ; d < zm.dimensions.size(); ++d) {
        char buf[10];
        sprintf(buf,"[%c]", 'a'+(int)d);
        ret += buf;
    }
    return ret;
}

/** Make an accessor that points to the last array **/
static string makeAccessorArray(const ZCMMember& zm, const string& obj)
{
    string ret;
    ret += obj + (obj.size() == 0 ? "" : ".") + zm.membername;
    for (size_t d = 0 ; d < zm.dimensions.size()-1; ++d) {
        char buf[10];
        sprintf(buf,"[%c]", 'a'+(int)d);
        ret += buf;
    }
    return ret;
}

static bool structHasStringMember(const ZCMStruct& zs)
{
    for (auto& zm : zs.members)
        if (zm.type.fullname == "string")
            return true;
    return false;
}

static string dimSizePrefix(const string& dimSize)
{
    char* eptr;
    int ret = strtol(dimSize.c_str(), &eptr, 0);
    (void) ret;
    if(*eptr == '\0')
        return "";
    else
        return "this.";
}

struct TypeTable
{
    std::unordered_map<string, PrimInfo> tbl;

    TypeTable()
    {
        tbl.emplace("byte", PrimInfo{
            "byte",
            "# = ins.readByte@($);",
            "outs.writeByte(#);"});

        tbl.emplace("int8_t", PrimInfo{
            "byte",
            "# = ins.readByte@($);",
            "outs.writeByte(#);"});

        tbl.emplace("int16_t", PrimInfo{
            "short",
            "# = ins.readShort@($);",
            "outs.writeShort(#);"});

        tbl.emplace("int32_t", PrimInfo{
            "int",
            "# = ins.readInt@($);",
            "outs.writeInt(#);"});

        tbl.emplace("int64_t", PrimInfo{
            "long",
            "# = ins.readLong@($);",
            "outs.writeLong(#);"});

        tbl.emplace("string", PrimInfo{
            "String",
            "__strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; ++_i) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); # = new String(__strbuf);",
            "__strbuf = new char[#.length()]; #.getChars(0, #.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; ++_i) outs.write(__strbuf[_i]); outs.writeByte(0);"});

        tbl.emplace("boolean", PrimInfo{
            "boolean",
            "# = ins.readByte()!=0;",
            "outs.writeByte( # ? 1 : 0);"});

        tbl.emplace("float", PrimInfo{
            "float",
            "# = ins.readFloat();",
            "outs.writeFloat(#);"});

        tbl.emplace("double", PrimInfo{
            "double",
            "# = ins.readDouble();",
            "outs.writeDouble(#);"});
    }

    PrimInfo* find(const string& type)
    {
        auto it = tbl.find(type);
        if (it == tbl.end())
            return nullptr;
        else
            return &it->second;
    }
};

static TypeTable typeTable;

struct EmitStruct : public Emitter
{
    const ZCMGen& zcm;
    const ZCMStruct& zs;

    EmitStruct(const ZCMGen& zcm, const ZCMStruct& zs, const string& fname):
        Emitter(fname), zcm(zcm), zs(zs) {}

    void encodeRecursive(const ZCMMember& zm, PrimInfo* pinfo, const string& accessor, int depth)
    {
        int ndims = (int)zm.dimensions.size();

        // base case: primitive array
        if (depth+1 == ndims && zm.type.numbits == 0 && pinfo != nullptr) {
            string accessorArray = makeAccessorArray(zm, "");
            if (pinfo->storage == "byte") {
                auto& dim = zm.dimensions[depth];
                if (dim.mode == ZCM_VAR) {
                    emit(2+depth, "if (this.%s > 0)", dim.size.c_str());
                    emit(3+depth, "outs.write(this.%s, 0, %s);", accessorArray.c_str(), dim.size.c_str());
                } else {
                    emit(2+depth, "outs.write(this.%s, 0, %s);", accessorArray.c_str(), dim.size.c_str());
                }
                return;
            }
        }

        // base case: generic
        if (depth == ndims) {
            if (pinfo) {
                if (zm.type.numbits == 0) {
                    emit(2 + ndims, "%s", specialReplace(pinfo->encode, accessor).c_str());
                } else {
                    // base case: bitfield
                    emit(2 + ndims, "outs.writeBits(%s, %u);", accessor.c_str(), zm.type.numbits);
                }
            } else {
                emit(2 + ndims, "%s", specialReplace("#._encodeRecursive(outs);", accessor).c_str());
            }
            return;
        }

        auto& dim = zm.dimensions[depth];
        emit(2+depth, "for (int %c = 0; %c < %s%s; ++%c) {",
             'a'+depth, 'a'+depth, dimSizePrefix(dim.size).c_str(), dim.size.c_str(), 'a'+depth);

        encodeRecursive(zm, pinfo, accessor, depth+1);

        emit(2+depth, "}");
    }

    void decodeRecursive(const ZCMMember& zm, PrimInfo* pinfo, const string& accessor, int depth)
    {
        int ndims = (int)zm.dimensions.size();

        // base case: primitive array
        if (depth+1 == ndims && zm.type.numbits == 0 && pinfo != nullptr) {
            string accessorArray = makeAccessorArray(zm, "");

            // byte array
            if (pinfo->storage == "byte") {
                auto& dim = zm.dimensions[depth];
                emit(2+depth, "ins.readFully(this.%s, 0, %s);", accessorArray.c_str(), dim.size.c_str());
                return;
            }
        }

        // base case: generic
        if (depth == ndims) {
            if (pinfo) {
                if (zm.type.numbits == 0) {
                    emit(2 + ndims, "%s", specialReplace(pinfo->decode, accessor).c_str());
                } else {
                    string readArg = std::to_string(zm.type.numbits);
                    readArg += zm.type.signExtend ? ", true" : ", false";
                    // base case: bitfield
                    emit(2 + ndims, "%s", specialReplace(pinfo->decode, accessor, "Bits",
                                                         readArg).c_str());
                }
            } else {
                emit(2 + ndims, "%s = %s._decodeRecursiveFactory(ins);",
                                accessor.c_str(), makeFqn(zcm, zm.type.fullname).c_str());
            }
            return;
        }

        auto& dim = zm.dimensions[depth];
        emit(2+depth, "for (int %c = 0; %c < %s%s; ++%c) {",
             'a'+depth, 'a'+depth, dimSizePrefix(dim.size).c_str(), dim.size.c_str(), 'a'+depth);

        decodeRecursive(zm, pinfo, accessor, depth+1);

        emit(2+depth, "}");
    }

    void copyRecursive(const ZCMMember& zm, PrimInfo* pinfo, const string& accessor, int depth)
    {
        int ndims = (int)zm.dimensions.size();

        // base case: primitive array
        if (depth+1 == ndims && pinfo != nullptr) {
            string accessorArray = makeAccessorArray(zm, "");

            // one method works for all primitive types, yay!
            auto& dim = zm.dimensions[depth];

            if (dim.mode == ZCM_VAR) {
                emit(2+depth, "if (this.%s > 0)", dim.size.c_str());
                emitStart(3+depth, "System.arraycopy(this.%s, 0, outobj.%s, 0, %s%s);",
                          accessorArray.c_str(),
                          accessorArray.c_str(),
                          dimSizePrefix(dim.size).c_str(),
                          dim.size.c_str());
            } else {
                emitStart(2+depth, "System.arraycopy(this.%s, 0, outobj.%s, 0, %s%s);",
                          accessorArray.c_str(),
                          accessorArray.c_str(),
                          dimSizePrefix(dim.size).c_str(),
                          dim.size.c_str());
            }

            return;
        }

        // base case: generic
        if (depth == ndims) {
            if (pinfo) {

                emitStart(2+ndims, "outobj.%s", zm.membername.c_str());
                for (int i = 0; i < ndims; ++i)
                    emitContinue("[%c]", 'a'+i);

                emitContinue(" = this.%s", zm.membername.c_str());
                for (int i = 0; i < ndims; ++i)
                    emitContinue("[%c]", 'a'+i);

                emitEnd(";");

            } else {
                emit(2+depth, "outobj.%s = this.%s.copy();", accessor.c_str(), accessor.c_str());
            }

            return;
        }

        auto& dim = zm.dimensions[depth];
        emit(2+depth, "for (int %c = 0; %c < %s%s; ++%c) {",
             'a'+depth, 'a'+depth, dimSizePrefix(dim.size).c_str(), dim.size.c_str(), 'a'+depth);

        copyRecursive(zm, pinfo, accessor, depth+1);

        emit(2+depth, "}");
    }

    void emitStruct()
    {
        emit(0, "/* ZCM type definition class file");
        emit(0, " * This file was automatically generated by zcm-gen");
        emit(0, " * DO NOT MODIFY BY HAND!!!!");
        emit(0, " */");
        emit(0, "");

        string package = "";
        if (zcm.gopt->wasSpecified("jpkgprefix"))
            package += zcm.gopt->getString("jpkgprefix");
        if (zs.structname.package.size() > 0)
        {
            if (package != "") package += ".";
            package += zs.structname.package;
        }

        emit(0, "package %s;", package.c_str());
        emit(0, " ");
        emit(0, "import java.io.*;");
        emit(0, "import java.util.*;");
        emit(0, "import zcm.zcm.*;");
        emit(0, " ");
        emit(0, "public final class %s %s", zs.structname.shortname.c_str(), zcm.gopt->getString("jdecl").c_str());
        emit(0, "{");

        for (auto& zm : zs.members) {
            PrimInfo* pinfo = typeTable.find(zm.type.fullname);
            emitStart(1, "public ");

            if (pinfo)  {
                emitContinue("%s", pinfo->storage.c_str());
            } else {
                emitContinue("%s", makeFqn(zcm, zm.type.fullname).c_str());
            }
            emitContinue(" %s", zm.membername.c_str());
            for (size_t i = 0; i < zm.dimensions.size(); ++i)
                emitContinue("[]");
            emitEnd(";");
        }
        emit(0," ");

        // public constructor
        emit(1,"public %s()", zs.structname.shortname.c_str());
        emit(1,"{");

        // pre-allocate any fixed-size arrays.
        for (auto& zm : zs.members) {
            PrimInfo* pinfo = typeTable.find(zm.type.fullname);

            int ndims = (int)zm.dimensions.size();
            if (ndims == 0 || !zm.isConstantSizeArray())
                continue;

            emitStart(2, "%s = new ", zm.membername.c_str());
            if (pinfo)
                emitContinue("%s", pinfo->storage.c_str());
            else
                emitContinue("%s", makeFqn(zcm, zm.type.fullname).c_str());

            for (auto& dim : zm.dimensions)
                emitContinue("[%s]", dim.size.c_str());
            emitEnd(";");
        }
        emit(1,"}");
        emit(0," ");

        emit(1, "public static final long ZCM_FINGERPRINT;");
        emit(1, "public static final long ZCM_FINGERPRINT_BASE = 0x%016" PRIx64 "L;", zs.hash);
        emit(0," ");

        //////////////////////////////////////////////////////////////
        // CONSTANTS

        emit(1, "public static final boolean IS_LITTLE_ENDIAN = %s;",
                zcm.gopt->getBool("little-endian-encoding") ? "true" : "false");
        for (auto& zc : zs.constants) {
            assert(ZCMGen::isLegalConstType(zc.type));

            auto& tn = zc.type;
            auto* name = zc.membername.c_str();
            auto* value = zc.valstr.c_str();

            if (tn == "int8_t" || tn == "byte") {
                emit(1, "public static final byte %s = (byte) %s;", name, value);
            } else if (tn == "int16_t") {
                emit(1, "public static final short %s = (short) %s;", name, value);
            } else if (tn == "int32_t") {
                emit(1, "public static final int %s = %s;", name, value);
            } else if (tn == "int64_t") {
                emit(1, "public static final long %s = %sL;", name, value);
            } else if (tn == "float") {
                emit(1, "public static final float %s = %sf;", name, value);
            } else if (tn == "double") {
                emit(1, "public static final double %s = %s;", name, value);
            } else if (tn == "string") {
                emit(1, "public static final String %s = %s;", name, value);
            } else {
                assert(0);
            }
        }
        if (zs.constants.size() > 0)
            emit(0, "");

        ///////////////// compute fingerprint //////////////////
        emit(1, "static {");
        emit(2, "ZCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());");
        emit(1, "}");
        emit(0, " ");

        emit(1, "public static long _hashRecursive(ArrayList<Class<?>> classes)");
        emit(1, "{");
        emit(2, "if (classes.contains(%s.class))", makeFqn(zcm, zs.structname.fullname).c_str());
        emit(3,     "return 0L;");
        emit(0, " ");
        emit(2, "classes.add(%s.class);", makeFqn(zcm, zs.structname.fullname).c_str());

        emit(2, "long hash = ZCM_FINGERPRINT_BASE");
        for (auto& zm : zs.members) {
            PrimInfo* pinfo = typeTable.find(zm.type.fullname);
            if (pinfo)
                continue;
            emit(3, " + %s._hashRecursive(classes)", makeFqn(zcm, zm.type.fullname).c_str());
        }
        emit(3,";");

        emit(2, "classes.remove(classes.size() - 1);");
        emit(2, "return (hash<<1) + ((hash>>>63)&1);");

        emit(1, "}");
        emit(0, " ");

        ///////////////// encode //////////////////

        emit(1,"public void encode(ZCMDataOutputStream outs) throws IOException");
        emit(1,"{");
        emit(2,"outs.writeLong(ZCM_FINGERPRINT);");
        emit(2,"_encodeRecursive(outs);");
        emit(1,"}");
        emit(0," ");

        emit(1,"public void _encodeRecursive(ZCMDataOutputStream outs) throws IOException");
        emit(1,"{");
        if (structHasStringMember(zs))
            emit(2, "char[] __strbuf = null;");

        bool inBitMode = false;
        size_t bitfieldNum = 0;
        for (auto& zm : zs.members) {
            if (!inBitMode && zm.type.numbits != 0) {
                inBitMode = true;
                emit(2, "// Start of bitfield %u", bitfieldNum);
                emit(0, "");
            } else if (inBitMode && zm.type.numbits == 0) {
                inBitMode = false;
                emit(2, "outs.resetBits();");
                emit(0, "");
                emit(2, "// End of bitfield %u", bitfieldNum);
                emit(0, "");
                ++bitfieldNum;
            }
            PrimInfo* pinfo = typeTable.find(zm.type.fullname);
            string accessor = makeAccessor(zm, "this");
            encodeRecursive(zm, pinfo, accessor, 0);
            emit(0," ");
        }
        if (inBitMode) {
            emit(2, "outs.resetBits();");
            emit(0, "");
            emit(2, "// End of bitfield %u", bitfieldNum);
        }
        emit(1,"}");
        emit(0," ");

        ///////////////// decode //////////////////
        auto* sn = zs.structname.shortname.c_str();
        auto fqn_ = makeFqn(zcm, zs.structname.fullname);
        auto* fqn = fqn_.c_str();

        // decoding constructors
        emit(1, "public %s(byte[] data) throws IOException", sn);
        emit(1, "{");
        emit(2, "this(new ZCMDataInputStream(data));");
        emit(1, "}");
        emit(0, " ");
        emit(1,"public %s(ZCMDataInputStream ins) throws IOException", sn);
        emit(1,"{");
        emit(2,"if (ins.readLong() != ZCM_FINGERPRINT)");
        emit(3,     "throw new IOException(\"ZCM Decode error: bad fingerprint\");");
        emit(0," ");
        emit(2,"_decodeRecursive(ins);");
        emit(1,"}");
        emit(0," ");

        emit(1,"public static %s _decodeRecursiveFactory(ZCMDataInputStream ins) throws IOException", fqn);
        emit(1,"{");
        emit(2,"%s o = new %s();", fqn, fqn);
        emit(2,"o._decodeRecursive(ins);");
        emit(2,"return o;");
        emit(1,"}");
        emit(0," ");

        emit(1,"public void _decodeRecursive(ZCMDataInputStream ins) throws IOException");
        emit(1,"{");
        if (structHasStringMember(zs))
            emit(2, "char[] __strbuf = null;");

        inBitMode = false;
        bitfieldNum = 0;
        for (auto& zm : zs.members) {
            if (!inBitMode && zm.type.numbits != 0) {
                inBitMode = true;
                emit(2, "// Start of bitfield %u", bitfieldNum);
                emit(0, "");
            } else if (inBitMode && zm.type.numbits == 0) {
                inBitMode = false;
                emit(2, "ins.resetBits();");
                emit(0, "");
                emit(2, "// End of bitfield %u", bitfieldNum);
                emit(0, "");
                ++bitfieldNum;
            }
            PrimInfo* pinfo = typeTable.find(zm.type.fullname);
            string accessor = makeAccessor(zm, "this");

            // allocate an array if necessary
            if (zm.dimensions.size() > 0) {

                emitStart(2, "this.%s = new ", zm.membername.c_str());

                if (pinfo)
                    emitContinue("%s", pinfo->storage.c_str());
                else
                    emitContinue("%s", makeFqn(zcm, zm.type.fullname).c_str());

                for (auto& dim : zm.dimensions)
                    emitContinue("[(int) %s]", dim.size.c_str());
                emitEnd(";");
            }

            decodeRecursive(zm, pinfo, accessor, 0);
            emit(0," ");
        }
        if (inBitMode) {
            emit(2, "ins.resetBits();");
            emit(0, "");
            emit(2, "// End of bitfield %u", bitfieldNum);
        }

        emit(1,"}");
        emit(0," ");


        ///////////////// copy //////////////////
        string classname = makeFqn(zcm, zs.structname.fullname);
        emit(1,"public %s copy()", classname.c_str());
        emit(1,"{");
        emit(2,"%s outobj = new %s();", classname.c_str(), classname.c_str());

        for (auto& zm : zs.members) {
            PrimInfo* pinfo = typeTable.find(zm.type.fullname);
            string accessor = makeAccessor(zm, "");

            // allocate an array if necessary
            if (zm.dimensions.size() > 0) {

                emitStart(2, "outobj.%s = new ", zm.membername.c_str());

                if (pinfo)
                    emitContinue("%s", pinfo->storage.c_str());
                else
                    emitContinue("%s", makeFqn(zcm, zm.type.fullname).c_str());

                for (auto& dim : zm.dimensions)
                    emitContinue("[(int) %s]", dim.size.c_str());
                emitEnd(";");
            }

            copyRecursive(zm, pinfo, accessor, 0);
            emit(0," ");
        }

        emit(2,"return outobj;");
        emit(1,"}");
        emit(0," ");

        ////////
        emit(0, "}\n");
    }
};

int emitJava(const ZCMGen& zcm)
{
    if (zcm.gopt->getBool("little-endian-encoding")) {
        printf("Java does not currently support little endian encoding\n");
        return -1;
    }

    string jpath = zcm.gopt->getString("jpath");
    string jpathPrefix = jpath + (jpath.size() > 0 ? "/" : "");
    bool jmkdir = zcm.gopt->getBool("jmkdir");

    //////////////////////////////////////////////////////////////
    // STRUCTS
    for (auto& zs : zcm.structs) {
        if (!zcm.gopt->wasSpecified("jpkgprefix") &&
            zs.structname.fullname.find('.') == string::npos) {
            fprintf(stderr, "Please provide a java package for the output java classes -- "
                            "either via the \"--jpkgprefix\" flag or inside the type itself\n");
            return -2;
        }

        string classname = makeFqn(zcm, zs.structname.fullname);
        string path = jpathPrefix + dotsToSlashes(classname) + ".java";

        if (!zcm.needsGeneration(zs.zcmfile, path))
            continue;

        if (jmkdir)
            FileUtil::makeDirsForFile(path);

        EmitStruct E{zcm, zs, path};
        if (!E.good()) {
            printf("E.good() failed!\n");
            return -1;
        }
        E.emitStruct();
    }

    return 0;
}

vector<string> getFilepathsJava(const ZCMGen& zcm)
{
    vector<string> ret;

    string jpath = zcm.gopt->getString("jpath");
    string jpathPrefix = jpath + (jpath.size() > 0 ? "/" : "");

    for (auto& zs : zcm.structs) {
        if (!zcm.gopt->wasSpecified("jpkgprefix") &&
            zs.structname.fullname.find('.') == string::npos) {
            fprintf(stderr, "Please provide a java package for the output java classes -- "
                            "either via the \"--jpkgprefix\" flag or inside the type itself\n");
            return {};
        }

        string classname = makeFqn(zcm, zs.structname.fullname);
        string path = jpathPrefix + dotsToSlashes(classname) + ".java";

        ret.push_back(path);
    }

    return ret;
}

unordered_set<string> getReservedKeywordsJava()
{
    return { "abstract", "assert", "boolean", "break", "byte", "case", "catch",
             "char", "class", "const", "continue", "default", "double", "do",
             "else", "enum", "extends", "false", "final", "finally", "float",
             "for", "goto", "if", "implements", "import", "instanceof", "int",
             "interface", "long", "native", "new", "null", "package", "private",
             "protected", "public", "return", "short", "static", "strictfp",
             "super", "switch", "synchronized", "this", "throw", "throws",
             "transient", "true", "try", "void", "volatile", "while" };
}
