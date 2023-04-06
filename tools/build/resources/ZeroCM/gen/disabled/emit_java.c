#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#ifdef WIN32
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>
#include <assert.h>

#include <sys/stat.h>
#include <sys/types.h>

#include "zcmgen.h"

#include "getopt.h"

#ifdef WIN32
#include <zcm/windows/WinPorting.h>
#endif

#define INDENT(n) (4*(n))

#define emit_start(n, ...) do { fprintf(f, "%*s", INDENT(n), ""); fprintf(f, __VA_ARGS__); } while (0)
#define emit_continue(...) do { fprintf(f, __VA_ARGS__); } while (0)
#define emit_end(...) do { fprintf(f, __VA_ARGS__); fprintf(f, "\n"); } while (0)
#define emit(n, ...) do { fprintf(f, "%*s", INDENT(n), ""); fprintf(f, __VA_ARGS__); fprintf(f, "\n"); } while (0)

static char *dots_to_slashes(const char *s)
{
    char *p = strdup(s);

    for (char *t=p; *t!=0; t++)
        if (*t == '.')
            *t = G_DIR_SEPARATOR;

    return p;
}

static void make_dirs_for_file(const char *path)
{
#ifdef WIN32
    char *dirname = g_path_get_dirname(path);
    g_mkdir_with_parents(dirname, 0755);
    g_free(dirname);
#else
    int len = strlen(path);
    for (int i = 0; i < len; i++) {
        if (path[i]=='/') {
            char *dirpath = (char *) malloc(i+1);
            strncpy(dirpath, path, i);
            dirpath[i]=0;

            mkdir(dirpath, 0755);
            free(dirpath);

            i++; // skip the '/'
        }
    }
#endif
}

void setup_java_options(getopt_t *gopt)
{
    getopt_add_string(gopt, 0,   "jpath",     "",         "Java file destination directory");
    getopt_add_bool(gopt, 0,    "jmkdir",     1,         "Make java source directories automatically");
    getopt_add_string(gopt, 0,   "jdecl",      "implements zcm.zcm.ZCMEncodable", "String added to class declarations");
    getopt_add_string(gopt, 0,   "jdefaultpkg", "zcmtypes", "Default Java package if ZCM type has no package");
}

typedef struct
{
    char *storage;
    char *decode;
    char *encode;
} primitive_info_t;

static primitive_info_t *prim(char *storage, char *decode, char *encode)
{
    primitive_info_t *p = (primitive_info_t*) calloc(sizeof(primitive_info_t), 1);
    p->storage = storage;
    p->decode = decode;
    p->encode = encode;

    return p;
}

static int jdefaultpkg_warned = 0;

const char *make_fqn(zcmgen_t *zcm, const char *type_name)
{
    if (strchr(type_name, '.')!=NULL)
        return type_name;

    if (!jdefaultpkg_warned && !getopt_was_specified(zcm->gopt, "jdefaultpkg")) {
        printf("Notice: enclosing ZCM types without package into java namespace '%s'.\n", getopt_get_string(zcm->gopt, "jdefaultpkg"));
        jdefaultpkg_warned = 1;
    }

    return g_strdup_printf("%s.%s", getopt_get_string(zcm->gopt, "jdefaultpkg"), type_name);
}

/** # -> replace1
    @ -> replace2
**/
static void freplace(FILE *f, const char *haystack, const char *replace1)
{
    int len = strlen(haystack);

    for (int pos = 0; pos < len; pos++)
    {
        if (haystack[pos]=='#')
            fprintf(f, "%s", replace1);
        else
            fprintf(f, "%c", haystack[pos]);
    }
}

static void make_accessor(zcm_member_t *lm, const char *obj, char *s)
{
    int ndim = g_ptr_array_size(lm->dimensions);
    int pos = 0;
    s[0] = 0;

    pos += sprintf(s, "%s%s%s", obj, obj[0]==0 ? "" : ".", lm->membername);
    for (int d = 0 ; d < ndim; d++)
        pos += sprintf(&s[pos],"[%c]", 'a'+d);
}

/** Make an accessor that points to the last array **/
static void make_accessor_array(zcm_member_t *lm, const char *obj, char *s)
{
    int ndim = g_ptr_array_size(lm->dimensions);
    int pos = 0;
    s[0] = 0;

    pos += sprintf(s, "%s%s%s", obj, obj[0]==0 ? "" : ".", lm->membername);
    for (int d = 0 ; d < ndim - 1; d++)
        pos += sprintf(&s[pos],"[%c]", 'a'+d);
}

static int struct_has_string_member(zcm_struct_t *lr)
{
    for (unsigned int member = 0; member < g_ptr_array_size(lr->members); member++) {
        zcm_member_t *lm = (zcm_member_t *) g_ptr_array_index(lr->members, member);
        if(!strcmp("string", lm->type->lctypename))
            return 1;
    }
    return 0;
}

static const char * dim_size_prefix(const char *dim_size) {
    char *eptr = NULL;
    long asdf = strtol(dim_size, &eptr, 0);
    (void) asdf;  // suppress compiler warnings
    if(*eptr == '\0')
        return "";
    else
        return "this.";
}

void encode_recursive(zcmgen_t *zcm, zcm_member_t *lm, FILE *f, primitive_info_t *pinfo, char *accessor, int depth)
{
    // base case: primitive array
    if (depth+1 == g_ptr_array_size(lm->dimensions) && pinfo != NULL) {
        char accessor_array[1024];
        make_accessor_array(lm, "", accessor_array);

        if (!strcmp(pinfo->storage, "byte")) {
            zcm_dimension_t *dim = (zcm_dimension_t*) g_ptr_array_index(lm->dimensions, depth);
            if (dim->mode == ZCM_VAR) {
                emit(2+depth, "if (this.%s > 0)", dim->size);
                emit(3+depth, "outs.write(this.%s, 0, %s);", accessor_array, dim->size);
            } else {
                emit(2+depth, "outs.write(this.%s, 0, %s);", accessor_array, dim->size);
            }
            return;
        }

        // some other kind of primitive array.
        // This seems to be slower than the default (and is untested for correctness), hence it is disabled.
        if (0 && !strcmp(pinfo->storage, "float")) {

            zcm_dimension_t *dim = (zcm_dimension_t*) g_ptr_array_index(lm->dimensions, depth);

            emit(2+depth, "{ ByteBuffer bbuf = ByteBuffer.allocate(%s%s*4); bbuf.order(ByteOrder.BIG_ENDIAN); ", dim_size_prefix(dim->size), dim->size);
            emit(2+depth, "  bbuf.asFloatBuffer().put(this.%s);", accessor_array);
            emit(2+depth, "  outs.write(bbuf.array()); }");
            return;
        }
    }


    // base case: generic
    if (depth == g_ptr_array_size(lm->dimensions)) {
        emit_start(2 + g_ptr_array_size(lm->dimensions),"");
        if (pinfo != NULL)
            freplace(f, pinfo->encode, accessor);
        else
            freplace(f, "#._encodeRecursive(outs);", accessor);
        emit_end(" ");

        return;
    }

    zcm_dimension_t *dim = (zcm_dimension_t*) g_ptr_array_index(lm->dimensions, depth);

    emit(2+depth, "for (int %c = 0; %c < %s%s; %c++) {",
         'a'+depth, 'a'+depth, dim_size_prefix(dim->size), dim->size, 'a'+depth);

    encode_recursive(zcm, lm, f, pinfo, accessor, depth+1);

    emit(2+depth, "}");
}

void decode_recursive(zcmgen_t *zcm, zcm_member_t *lm, FILE *f, primitive_info_t *pinfo, char *accessor, int depth)
{
    // base case: primitive array
    if (depth+1 == g_ptr_array_size(lm->dimensions) && pinfo != NULL) {

        char accessor_array[1024];
        make_accessor_array(lm, "", accessor_array);

        // byte array
        if (!strcmp(pinfo->storage, "byte")) {
            zcm_dimension_t *dim = (zcm_dimension_t*) g_ptr_array_index(lm->dimensions, depth);
            emit_start(2+depth, "ins.readFully(this.%s, 0, %s);", accessor_array, dim->size);
            return;
        }

        // some other kind of primitive array.
        // This seems to be slower than the default (and is untested for correctness), hence it is disabled.
        if (0 && !strcmp(pinfo->storage, "float")) {
            zcm_dimension_t *dim = (zcm_dimension_t*) g_ptr_array_index(lm->dimensions, depth);

            emit(2+depth, "{ byte bb[] = new byte[%s%s*4]; ins.readFully(bb); ByteBuffer bbuf = ByteBuffer.wrap(bb); bbuf.order(ByteOrder.BIG_ENDIAN); ", dim_size_prefix(dim->size), dim->size);
            emit(2+depth, "  bbuf.asFloatBuffer().get(this.%s); }", accessor_array);
            return;
        }
  }

    // base case: generic
    if (depth == g_ptr_array_size(lm->dimensions)) {
        emit_start(2 + g_ptr_array_size(lm->dimensions),"");
        if (pinfo != NULL)
            freplace(f, pinfo->decode, accessor);
        else {
            emit_continue("%s = %s._decodeRecursiveFactory(ins);", accessor, make_fqn(zcm, lm->type->lctypename));
        }
        emit_end("");

        return;
    }

    zcm_dimension_t *dim = (zcm_dimension_t*) g_ptr_array_index(lm->dimensions, depth);

    emit(2+depth, "for (int %c = 0; %c < %s%s; %c++) {",
         'a'+depth, 'a'+depth, dim_size_prefix(dim->size), dim->size, 'a'+depth);

    decode_recursive(zcm, lm, f, pinfo, accessor, depth+1);

    emit(2+depth, "}");
}

void copy_recursive(zcmgen_t *zcm, zcm_member_t *lm, FILE *f, primitive_info_t *pinfo, char *accessor, int depth)
{
    // base case: primitive array
    if (depth+1 == g_ptr_array_size(lm->dimensions) && pinfo != NULL) {

        char accessor_array[1024];
        make_accessor_array(lm, "", accessor_array);

        // one method works for all primitive types, yay!
        zcm_dimension_t *dim = (zcm_dimension_t*) g_ptr_array_index(lm->dimensions, depth);

        if (dim->mode == ZCM_VAR) {
            emit(2+depth, "if (this.%s > 0)", dim->size);
            emit_start(3+depth, "System.arraycopy(this.%s, 0, outobj.%s, 0, %s%s);",
                       accessor_array,
                       accessor_array,
                       dim_size_prefix(dim->size),
                       dim->size);
        } else {
            emit_start(2+depth, "System.arraycopy(this.%s, 0, outobj.%s, 0, %s%s);",
                       accessor_array,
                       accessor_array,
                       dim_size_prefix(dim->size),
                       dim->size);
        }

        return;
    }

    // base case: generic
    if (depth == g_ptr_array_size(lm->dimensions)) {
        if (pinfo != NULL) {

            emit_start(2+g_ptr_array_size(lm->dimensions), "outobj.%s", lm->membername);
            for (unsigned int i = 0; i < g_ptr_array_size(lm->dimensions); i++) {
                emit_continue("[%c]", 'a'+i);
            }
            emit_continue(" = this.%s", lm->membername);

            for (unsigned int i = 0; i < g_ptr_array_size(lm->dimensions); i++) {
                emit_continue("[%c]", 'a'+i);
            }

            emit_end(";");

        } else {
            emit(2+depth, "outobj.%s = this.%s.copy();", accessor, accessor);
        }

        return;
    }

    zcm_dimension_t *dim = (zcm_dimension_t*) g_ptr_array_index(lm->dimensions, depth);

    emit(2+depth, "for (int %c = 0; %c < %s%s; %c++) {",
         'a'+depth, 'a'+depth, dim_size_prefix(dim->size), dim->size, 'a'+depth);

    copy_recursive(zcm, lm, f, pinfo, accessor, depth+1);

    emit(2+depth, "}");
}

int emit_java(zcmgen_t *zcm)
{
    GHashTable *type_table = g_hash_table_new(g_str_hash, g_str_equal);

    g_hash_table_insert(type_table, "byte",   prim("byte",
                                             "# = ins.readByte();",
                                             "outs.writeByte(#);"));
    g_hash_table_insert(type_table, "int8_t",   prim("byte",
                                               "# = ins.readByte();",
                                               "outs.writeByte(#);"));
    g_hash_table_insert(type_table, "int16_t",  prim("short",
                                               "# = ins.readShort();",
                                               "outs.writeShort(#);"));
    g_hash_table_insert(type_table, "int32_t",  prim("int",
                                               "# = ins.readInt();",
                                               "outs.writeInt(#);"));
    g_hash_table_insert(type_table, "int64_t",  prim("long",
                                               "# = ins.readLong();",
                                               "outs.writeLong(#);"));

    g_hash_table_insert(type_table, "string",   prim("String",
                                                     "__strbuf = new char[ins.readInt()-1]; for (int _i = 0; _i < __strbuf.length; _i++) __strbuf[_i] = (char) (ins.readByte()&0xff); ins.readByte(); # = new String(__strbuf);",
                                                     "__strbuf = new char[#.length()]; #.getChars(0, #.length(), __strbuf, 0); outs.writeInt(__strbuf.length+1); for (int _i = 0; _i < __strbuf.length; _i++) outs.write(__strbuf[_i]); outs.writeByte(0);"));

//    g_hash_table_insert(type_table, "string",   prim("String",
//                                               "__strbuf = new byte[ins.readInt()-1]; ins.readFully(__strbuf); ins.readByte(); # = new String(__strbuf, \"UTF-8\");",
//                                               "__strbuf = #.getBytes(\"UTF-8\"); outs.writeInt(__strbuf.length+1); outs.write(__strbuf, 0, __strbuf.length); outs.writeByte(0);"));

    g_hash_table_insert(type_table, "boolean",  prim("boolean",
                                               "# = ins.readByte()!=0;",
                                               "outs.writeByte( # ? 1 : 0);"));
    g_hash_table_insert(type_table, "float",    prim("float",
                                               "# = ins.readFloat();",
                                               "outs.writeFloat(#);"));
    g_hash_table_insert(type_table, "double",   prim("double",
                                               "# = ins.readDouble();",
                                               "outs.writeDouble(#);"));

    //////////////////////////////////////////////////////////////
    // ENUMS
    for (unsigned int en = 0; en < g_ptr_array_size(zcm->enums); en++) {
        zcm_enum_t *le = (zcm_enum_t *) g_ptr_array_index(zcm->enums, en);

        const char *classname = make_fqn(zcm, le->enumname->lctypename);
        char *path = g_strdup_printf("%s%s%s.java",
                                  getopt_get_string(zcm->gopt, "jpath"),
                                  strlen(getopt_get_string(zcm->gopt, "jpath")) > 0 ? G_DIR_SEPARATOR_S : "",
                                  dots_to_slashes(classname));

        if (!zcm_needs_generation(zcm, le->zcmfile, path))
            continue;

        if (getopt_get_bool(zcm->gopt, "jmkdir"))
            make_dirs_for_file(path);

        FILE *f = fopen(path, "w");
        if (f==NULL)
            return -1;

        if (strlen(le->enumname->package) > 0)
            emit(0, "package %s;", le->enumname->package);
        else
            emit(0, "package %s;", getopt_get_string(zcm->gopt, "jdefaultpkg"));

        emit(0, " ");
        emit(0, "import java.io.*;");
        emit(0, "import java.util.*;");
        emit(0, " ");

        emit(0, "public final class %s %s", le->enumname->shortname, getopt_get_string(zcm->gopt, "jdecl"));

        emit(0, "{");
        emit(1, "public int value;");
        emit(0, " ");

        for (unsigned int v = 0; v < g_ptr_array_size(le->values); v++) {
            zcm_enum_value_t *lev = (zcm_enum_value_t *) g_ptr_array_index(le->values, v);
            emit(1, "public static final int %-16s = %i;",
                    lev->valuename, lev->value);
        }
        emit(0," ");

        emit(1,"public %s(int value) { this.value = value; }",
                le->enumname->shortname);
        emit(0," ");

        emit(1,"public int getValue() { return value; }");
        emit(0," ");

        emit(1,"public void _encodeRecursive(DataOutput outs) throws IOException");
        emit(1,"{");
        emit(2,"outs.writeInt(this.value);");
        emit(1,"}");
        emit(0," ");

        emit(1,"public void encode(DataOutput outs) throws IOException");
        emit(1,"{");
        emit(2,"outs.writeLong(ZCM_FINGERPRINT);");
        emit(2,"_encodeRecursive(outs);");
        emit(1,"}");
        emit(0," ");

        emit(1,"public static %s _decodeRecursiveFactory(DataInput ins) throws IOException", make_fqn(zcm, le->enumname->lctypename));
        emit(1,"{");
        emit(2,"%s o = new %s(0);", make_fqn(zcm, le->enumname->lctypename), make_fqn(zcm, le->enumname->lctypename));
        emit(2,"o._decodeRecursive(ins);");
        emit(2,"return o;");
        emit(1,"}");
        emit(0," ");

        emit(1,"public void _decodeRecursive(DataInput ins) throws IOException");
        emit(1,"{");
        emit(2,"this.value = ins.readInt();");
        emit(1,"}");
        emit(0," ");

        emit(1,"public %s(DataInput ins) throws IOException", le->enumname->shortname);
        emit(1,"{");
        emit(2,"long hash = ins.readLong();");
        emit(2,"if (hash != ZCM_FINGERPRINT)");
        emit(3,     "throw new IOException(\"ZCM Decode error: bad fingerprint\");");
        emit(2,"_decodeRecursive(ins);");
        emit(1,"}");
        emit(0," ");

        emit(1,"public %s copy()", classname);
        emit(1,"{");
        emit(2,"return new %s(this.value);", classname);
        emit(1,"}");
        emit(0," ");

        emit(1,"public static final long _hashRecursive(ArrayList<Class<?>> clss)");
        emit(1,"{");
        emit(2,"return ZCM_FINGERPRINT;");
        emit(1,"}");
        emit(0," ");
        emit(1, "public static final long ZCM_FINGERPRINT = 0x%016"PRIx64"L;", le->hash);
        emit(0, "}");
        fclose(f);
    }

    for (unsigned int st = 0; st < g_ptr_array_size(zcm->structs); st++) {
        zcm_struct_t *lr = (zcm_struct_t *) g_ptr_array_index(zcm->structs, st);

        const char *classname = make_fqn(zcm, lr->structname->lctypename);
        char *path = g_strdup_printf("%s%s%s.java",
                                  getopt_get_string(zcm->gopt, "jpath"),
                                  strlen(getopt_get_string(zcm->gopt, "jpath")) > 0 ? G_DIR_SEPARATOR_S : "",
                                  dots_to_slashes(classname));

        if (!zcm_needs_generation(zcm, lr->zcmfile, path))
            continue;

        if (getopt_get_bool(zcm->gopt, "jmkdir"))
            make_dirs_for_file(path);

        FILE *f = fopen(path, "w");
        if (f==NULL)
            return -1;

        emit(0, "/* ZCM type definition class file\n"
                " * This file was automatically generated by zcm-gen\n"
                " * DO NOT MODIFY BY HAND!!!!\n"
                " */\n");

        if (strlen(lr->structname->package) > 0)
            emit(0, "package %s;", lr->structname->package);
        else
            emit(0, "package %s;", getopt_get_string(zcm->gopt, "jdefaultpkg"));

        emit(0, " ");
        emit(0, "import java.io.*;");

        if (0) {
            // Determine if we even need the java.nio.* package.
            int usenio = 0;
            for (unsigned int member = 0; member < g_ptr_array_size(lr->members); member++) {
                zcm_member_t *lm = (zcm_member_t *) g_ptr_array_index(lr->members, member);
                primitive_info_t *pinfo = (primitive_info_t*) g_hash_table_lookup(type_table, lm->type->lctypename);
                if (pinfo!=NULL && !strcmp(pinfo->storage, "float")) {
                    usenio = 1;
                    break;
                }
            }
            if (usenio)
                emit(0, "import java.nio.*;");
        }

        emit(0, "import java.util.*;");
        emit(0, "import zcm.zcm.*;");
        emit(0, " ");
        emit(0, "public final class %s %s", lr->structname->shortname, getopt_get_string(zcm->gopt, "jdecl"));
        emit(0, "{");

        for (unsigned int member = 0; member < g_ptr_array_size(lr->members); member++) {
            zcm_member_t *lm = (zcm_member_t *) g_ptr_array_index(lr->members, member);
            primitive_info_t *pinfo = (primitive_info_t*) g_hash_table_lookup(type_table, lm->type->lctypename);

            emit_start(1, "public ");

            if (pinfo==NULL)  {
                emit_continue("%s", make_fqn(zcm, lm->type->lctypename));
            } else {
                emit_continue("%s", pinfo->storage);
            }

            emit_continue(" %s", lm->membername);
            for (unsigned int i = 0; i < g_ptr_array_size(lm->dimensions); i++)
                emit_continue("[]");
            emit_end(";");
        }
        emit(0," ");

        // public constructor
        emit(1,"public %s()", lr->structname->shortname);
        emit(1,"{");

        // pre-allocate any fixed-size arrays.
        for (unsigned int member = 0; member < g_ptr_array_size(lr->members); member++) {
            zcm_member_t *lm = (zcm_member_t *) g_ptr_array_index(lr->members, member);
            primitive_info_t *pinfo = (primitive_info_t*) g_hash_table_lookup(type_table, lm->type->lctypename);

            if (g_ptr_array_size(lm->dimensions)==0 || !zcm_is_constant_size_array(lm))
                continue;

            emit_start(2, "%s = new ", lm->membername);
            if (pinfo != NULL)
                emit_continue("%s", pinfo->storage);
            else
                emit_continue("%s", make_fqn(zcm, lm->type->lctypename));

            for (unsigned int i = 0; i < g_ptr_array_size(lm->dimensions); i++) {
                zcm_dimension_t *dim = (zcm_dimension_t*) g_ptr_array_index(lm->dimensions, i);
                emit_continue("[%s]", dim->size);
            }
            emit_end(";");
        }
        emit(1,"}");
        emit(0," ");

        emit(1, "public static final long ZCM_FINGERPRINT;");
        emit(1, "public static final long ZCM_FINGERPRINT_BASE = 0x%016"PRIx64"L;", lr->hash);
        emit(0," ");

        //////////////////////////////////////////////////////////////
        // CONSTANTS
        for (unsigned int cn = 0; cn < g_ptr_array_size(lr->constants); cn++) {
            zcm_constant_t *lc = (zcm_constant_t *) g_ptr_array_index(lr->constants, cn);
            assert(zcm_is_legal_const_type(lc->lctypename));

            if (!strcmp(lc->lctypename, "int8_t")) {
                emit(1, "public static final byte %s = (byte) %s;", lc->membername, lc->val_str);
            } else if (!strcmp(lc->lctypename, "int16_t")) {
                emit(1, "public static final short %s = (short) %s;", lc->membername, lc->val_str);
            } else if (!strcmp(lc->lctypename, "int32_t")) {
                emit(1, "public static final int %s = %s;", lc->membername, lc->val_str);
            } else if (!strcmp(lc->lctypename, "int64_t")) {
                emit(1, "public static final long %s = %sL;", lc->membername, lc->val_str);
            } else if (!strcmp(lc->lctypename, "float")) {
                emit(1, "public static final float %s = %sf;", lc->membername, lc->val_str);
            } else if (!strcmp(lc->lctypename, "double")) {
                emit(1, "public static final double %s = %s;", lc->membername, lc->val_str);
            } else {
                assert(0);
            }
        }
        if (g_ptr_array_size(lr->constants) > 0)
            emit(0, "");

        ///////////////// compute fingerprint //////////////////
        emit(1, "static {");
        emit(2, "ZCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());");
        emit(1, "}");
        emit(0, " ");

        emit(1, "public static long _hashRecursive(ArrayList<Class<?>> classes)");
        emit(1, "{");
        emit(2, "if (classes.contains(%s.class))", make_fqn(zcm, lr->structname->lctypename));
        emit(3,     "return 0L;");
        emit(0, " ");
        emit(2, "classes.add(%s.class);", make_fqn(zcm, lr->structname->lctypename));

        emit(2, "long hash = ZCM_FINGERPRINT_BASE");
        for (unsigned int member = 0; member < g_ptr_array_size(lr->members); member++) {
            zcm_member_t *lm = (zcm_member_t *) g_ptr_array_index(lr->members, member);
            primitive_info_t *pinfo = (primitive_info_t*) g_hash_table_lookup(type_table, lm->type->lctypename);

            if (pinfo)
                continue;

            emit(3, " + %s._hashRecursive(classes)", make_fqn(zcm, lm->type->lctypename));
        }
        emit(3,";");

        emit(2, "classes.remove(classes.size() - 1);");
        emit(2, "return (hash<<1) + ((hash>>63)&1);");

        emit(1, "}");
        emit(0, " ");

        ///////////////// encode //////////////////

        emit(1,"public void encode(DataOutput outs) throws IOException");
        emit(1,"{");
        emit(2,"outs.writeLong(ZCM_FINGERPRINT);");
        emit(2,"_encodeRecursive(outs);");
        emit(1,"}");
        emit(0," ");

        emit(1,"public void _encodeRecursive(DataOutput outs) throws IOException");
        emit(1,"{");
        if(struct_has_string_member(lr))
            emit(2, "char[] __strbuf = null;");
        char accessor[1024];

        for (unsigned int member = 0; member < g_ptr_array_size(lr->members); member++) {
            zcm_member_t *lm = (zcm_member_t *) g_ptr_array_index(lr->members, member);
            primitive_info_t *pinfo = (primitive_info_t*) g_hash_table_lookup(type_table, lm->type->lctypename);
            make_accessor(lm, "this", accessor);

            encode_recursive(zcm, lm, f, pinfo, accessor, 0);
            emit(0," ");
        }
        emit(1,"}");
        emit(0," ");

        ///////////////// decode //////////////////

        // decoding constructors
        emit(1, "public %s(byte[] data) throws IOException", lr->structname->shortname);
        emit(1, "{");
        emit(2, "this(new ZCMDataInputStream(data));");
        emit(1, "}");
        emit(0, " ");

        emit(1,"public %s(DataInput ins) throws IOException", lr->structname->shortname);
        emit(1,"{");
        emit(2,"if (ins.readLong() != ZCM_FINGERPRINT)");
        emit(3,     "throw new IOException(\"ZCM Decode error: bad fingerprint\");");
        emit(0," ");
        emit(2,"_decodeRecursive(ins);");
        emit(1,"}");
        emit(0," ");

        emit(1,"public static %s _decodeRecursiveFactory(DataInput ins) throws IOException", make_fqn(zcm, lr->structname->lctypename));
        emit(1,"{");
        emit(2,"%s o = new %s();", make_fqn(zcm, lr->structname->lctypename), make_fqn(zcm, lr->structname->lctypename));
        emit(2,"o._decodeRecursive(ins);");
        emit(2,"return o;");
        emit(1,"}");
        emit(0," ");

        emit(1,"public void _decodeRecursive(DataInput ins) throws IOException");
        emit(1,"{");
        if(struct_has_string_member(lr))
            emit(2, "char[] __strbuf = null;");
        for (unsigned int member = 0; member < g_ptr_array_size(lr->members); member++) {
            zcm_member_t *lm = (zcm_member_t *) g_ptr_array_index(lr->members, member);
            primitive_info_t *pinfo = (primitive_info_t*) g_hash_table_lookup(type_table, lm->type->lctypename);

            make_accessor(lm, "this", accessor);

            // allocate an array if necessary
            if (g_ptr_array_size(lm->dimensions) > 0) {

                emit_start(2, "this.%s = new ", lm->membername);

                if (pinfo != NULL)
                    emit_continue("%s", pinfo->storage);
                else
                    emit_continue("%s", make_fqn(zcm, lm->type->lctypename));

                for (unsigned int i = 0; i < g_ptr_array_size(lm->dimensions); i++) {
                    zcm_dimension_t *dim = (zcm_dimension_t*) g_ptr_array_index(lm->dimensions, i);
                    emit_continue("[(int) %s]", dim->size);
                }
                emit_end(";");
            }

            decode_recursive(zcm, lm, f, pinfo, accessor, 0);
            emit(0," ");
        }

        emit(1,"}");
        emit(0," ");


        ///////////////// copy //////////////////

        emit(1,"public %s copy()", classname);
        emit(1,"{");
        emit(2,"%s outobj = new %s();", classname, classname);

        for (unsigned int member = 0; member < g_ptr_array_size(lr->members); member++) {
            zcm_member_t *lm = (zcm_member_t *) g_ptr_array_index(lr->members, member);
            primitive_info_t *pinfo = (primitive_info_t*) g_hash_table_lookup(type_table, lm->type->lctypename);
            make_accessor(lm, "", accessor);

            // allocate an array if necessary
            if (g_ptr_array_size(lm->dimensions) > 0) {

                emit_start(2, "outobj.%s = new ", lm->membername);

                if (pinfo != NULL)
                    emit_continue("%s", pinfo->storage);
                else
                    emit_continue("%s", make_fqn(zcm, lm->type->lctypename));

                for (unsigned int i = 0; i < g_ptr_array_size(lm->dimensions); i++) {
                    zcm_dimension_t *dim = (zcm_dimension_t*) g_ptr_array_index(lm->dimensions, i);
                    emit_continue("[(int) %s]", dim->size);
                }
                emit_end(";");
            }

            copy_recursive(zcm, lm, f, pinfo, accessor, 0);
            emit(0," ");
        }

        emit(2,"return outobj;");
        emit(1,"}");
        emit(0," ");

        ////////
        emit(0, "}\n");
        fclose(f);
    }

/* XXX deallocate our storage. unfinished since memory leaks are non-critical for zcm-gen.

    hashtable_iterator_t *hit = hashtable_iterator_create(type_table);
    hashtable_entry_t *entry;
    while ((entry = hashtable_iterator_next(hit)) != NULL) {
        free((char*) entry->value);
    }
    hashtable_iterator_destroy(hit);

    hashtable_destroy(type_table);
*/
    return 0;
}
