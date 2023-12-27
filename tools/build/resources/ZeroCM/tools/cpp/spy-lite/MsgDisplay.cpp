#include "MsgDisplay.hpp"

#define LINE_FMT_STR "    %-20.20s %-20.20s "
#define MAX_ARRAY_DISPLAY 100
#define MAX_ARRAY_ELT_PER_LINE 10

static inline bool is_ascii(int8_t c)
{
    return (32 <= c && c <= 126);
}
static void print_value_scalar(TypeDb& db, zcm_field_t *field, void *data, int *usertype_count)
{

    switch(field->type) {

        case ZCM_FIELD_BYTE: {
            uint8_t i = *(uint8_t *) data;
            printf(" %u", i);
            if (is_ascii(i)) printf(" (%c)", i);
            break;
        }

        case ZCM_FIELD_INT8_T: {
            int8_t i = *(int8_t *) data;
            printf(" %d", i);
            if (is_ascii(i)) printf(" (%c)", i);
            break;
        }

        case ZCM_FIELD_INT16_T:
            printf("% d", *(int16_t *) data);
            break;

        case ZCM_FIELD_INT32_T:
            printf("% d", *(int32_t *) data);
            break;

        case ZCM_FIELD_INT64_T:
            printf("% " PRIi64 "", *(int64_t *) data);
            break;

        case ZCM_FIELD_FLOAT:
            printf("% f", *(float *) data);
            break;

        case ZCM_FIELD_DOUBLE:
            printf("% f", *(double *) data);
            break;

        case ZCM_FIELD_STRING:
            printf("\"%s\"", *(const char **) data);
            break;

        case ZCM_FIELD_BOOLEAN:
            printf("%s", (*(int8_t*) data) == 1 ? "true" : "false");
            break;

        case ZCM_FIELD_USER_TYPE: {
            if (db.getByName(StringUtil::dotsToUnderscores(field->typestr))) {
                if(usertype_count == NULL) {
                    printf("<USER>");
                } else {
                    int n = ++*usertype_count;
                    printf("<%d>", n);
                }
            } else {
                printf("<unknown-user-type>");
            }
            break;
        }

        default:
            printf("???");
            fprintf(stderr, "ERR: failed to handle zcm message field type: %s\n", field->typestr);
            break;
    }
}

static size_t typesize(zcm_field_type_t type)
{
    switch(type) {
        case ZCM_FIELD_INT8_T:   return sizeof(int8_t);
        case ZCM_FIELD_INT16_T:  return sizeof(int16_t);
        case ZCM_FIELD_INT32_T:  return sizeof(int32_t);
        case ZCM_FIELD_INT64_T:  return sizeof(int64_t);
        case ZCM_FIELD_BYTE:     return sizeof(int8_t);
        case ZCM_FIELD_FLOAT:    return sizeof(float);
        case ZCM_FIELD_DOUBLE:   return sizeof(double);
        case ZCM_FIELD_STRING:   return sizeof(const char *);
        case ZCM_FIELD_BOOLEAN:  return sizeof(int8_t);

        case ZCM_FIELD_USER_TYPE:
        default:
            return 0;
    }
}

static void print_value_array(TypeDb& db, zcm_field_t *field, void *data, int *usertype_count)
{
    if(field->num_dim == 1) {
        int len = field->dim_size[0];
        size_t elt_size = typesize(field->type);
        void *p = (!field->dim_is_variable[0]) ? field->data : *(void **) field->data;

        printf("[");
        for(int i = 0; i < len; i++) {
            if(i != 0 && i % MAX_ARRAY_ELT_PER_LINE == 0) {
                printf("\n" LINE_FMT_STR " ", "", "");
            }
            if(i == MAX_ARRAY_DISPLAY) {
                printf("...more...");
                break;
            }
            print_value_scalar(db, field, p, usertype_count);
            if(i+1 != len)
                printf(", ");
            p = (void *)((uint8_t *) p + elt_size);
        }
        printf(" ]");
    } else {
        printf("<Multi-dim array: not yet supported>");
    }
}

static inline void strnfmtappend(char *buf, size_t sz, size_t *used, const char *fmt, ...)
{
    assert(*used <= sz);

    va_list va;
    va_start(va, fmt);
    int n = vsnprintf(buf+*used, sz-*used, fmt, va);
    va_end(va);

    *used += n;
    if(*used > sz)
        *used = sz;
}

void msg_display(TypeDb& db, const TypeMetadata& metadata_,
                 void *msg, const MsgDisplayState& state)
{
    const TypeMetadata *metadata = &metadata_;

    /* first, we need to resolve/recurse to the proper submessage */
    #define TRAVERSAL_BUFSZ 1024
    char traversal[TRAVERSAL_BUFSZ];
    size_t traversal_used = 0;

    vector<pair<uint8_t*, const zcm_type_info_t*>> decodedTypeBuf;

    auto cleanupDecodedTypeBuf = [&decodedTypeBuf](){
        for (auto d : decodedTypeBuf) {
            d.second->decode_cleanup(d.first);
            delete d.first;
        }
        decodedTypeBuf.clear();
    };

    auto try_byte_array_to_zcmtype = [&decodedTypeBuf,&db](zcm_field_t& field) {
        if (field.num_dim != 1 ||
            field.type != ZCM_FIELD_BYTE ||
            field.dim_size[0] < (int) sizeof(uint64_t))
            return;

        void *p = (!field.dim_is_variable[0]) ? field.data : *(void **) field.data;

        i64 hash = 0;
        __int64_t_decode_array(p, 0, field.dim_size[0], &hash, 1);

        const TypeMetadata* t = db.getByHash(hash);
        if (!t) return;

        uint8_t* msg = new uint8_t[t->info->struct_size()];
        assert(t->info->decode(*(void **) field.data, 0, field.dim_size[0], msg) > 0);
        decodedTypeBuf.push_back({msg, t->info});

        field.data = decodedTypeBuf.back().first;

        field.type = ZCM_FIELD_USER_TYPE;
        field.typestr = t->name.c_str();
        field.num_dim = 0;
        field.dim_size[0] = 0;
        field.dim_is_variable[0] = 0;
    };

    strnfmtappend(traversal, TRAVERSAL_BUFSZ, &traversal_used, "top");

    size_t i;
    for(i = 0; i < state.recur_table.size(); i++) {

        // get the desired <USER> id # to recurse on
        size_t recur_i = state.recur_table[i];

        // iterate through the fields until we find the corresponding one
        zcm_field_t field;
        const zcm_type_info_t *typeinfo = metadata->info;
        int num_fields = typeinfo->num_fields();
        size_t user_field_count = 0;
        int inside_array = 0;
        int index = 0;
        for(int j = 0; j < num_fields; j++) {
            typeinfo->get_field(msg, j, &field);
            try_byte_array_to_zcmtype(field);

            inside_array = 0;

            if(field.type == ZCM_FIELD_USER_TYPE) {

                // two possiblities here: 1) scalar or 2) array

                // 1) its a scalar
                if(field.num_dim == 0)  {
                    if(++user_field_count == recur_i)
                        break;
                }

                // 2) its an array
                else if(field.num_dim == 1) {
                    inside_array = 1;
                    for(index = 0; index < field.dim_size[0]; index++) {
                        if(++user_field_count == recur_i)
                            goto break_loop;
                    }
                }

                else {
                    //DEBUG(1, "NOTE: Multi-dim arrays not supported\n");
                }
            }
        }

    break_loop:
        // not found?
        if(user_field_count != recur_i)
            break;

        msg = field.data;
        metadata = db.getByName(StringUtil::dotsToUnderscores(field.typestr));
        if(metadata == NULL) {
            printf("ERROR: failed to find %s\n", field.typestr);
            cleanupDecodedTypeBuf();
            return;
        }

        strnfmtappend(traversal, TRAVERSAL_BUFSZ, &traversal_used,
                      " -> %s", field.name);

        if(inside_array) {

            // if its a variable array, we need to dereference the field
            // to get the actual array address
            if(field.dim_is_variable[0])
                msg = *(void **)msg;

            // compute the address of this index
            size_t typesz = metadata->info->struct_size();
            msg = (void*)((char*)msg + typesz * index);

            strnfmtappend(traversal, TRAVERSAL_BUFSZ, &traversal_used,
                          "[%d]", index);
        }

    }

    // sub-message recurse failed?
    if(i != state.recur_table.size()) {
        printf("ERROR: failed recurse to find sub-messages\n");
        cleanupDecodedTypeBuf();
        return;
    }

    const zcm_type_info_t *typeinfo = metadata->info;
    zcm_field_t field;
    int num_fields = typeinfo->num_fields();
    int usertype_count = 0;

    printf("         Traversal: %s \n", traversal);
    printf("   ----------------------------------------------------------------\n");

    for(int i = 0; i < num_fields; i++) {
        typeinfo->get_field(msg, i, &field);
        try_byte_array_to_zcmtype(field);

        printf(LINE_FMT_STR, field.name, field.typestr);

        if(field.num_dim == 0)
            print_value_scalar(db, &field, field.data, &usertype_count);
        else
            print_value_array(db, &field, field.data, &usertype_count);

        printf("\n");
    }
    cleanupDecodedTypeBuf();
}
