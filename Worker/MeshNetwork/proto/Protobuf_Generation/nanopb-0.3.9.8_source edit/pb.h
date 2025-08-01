/* Common parts of the nanopb library. Most of these are quite low-level
 * stuff. For the high-level interface, see pb_encode.h and pb_decode.h.
 */

#ifndef PB_H_INCLUDED
#define PB_H_INCLUDED

#include "progmem.h"

/*****************************************************************
 * Nanopb compilation time options. You can change these here by *
 * uncommenting the lines, or on the compiler command line.      *
 *****************************************************************/

/* Enable support for dynamically allocated fields */
/* #define PB_ENABLE_MALLOC 1 */

/* Define this if your CPU / compiler combination does not support
 * unaligned memory access to packed structures. */
/* #define PB_NO_PACKED_STRUCTS 1 */

/* Increase the number of required fields that are tracked.
 * A compiler warning will tell if you need this. */
/* #define PB_MAX_REQUIRED_FIELDS 256 */

/* Add support for tag numbers > 255 and fields larger than 255 bytes. */
#define PB_FIELD_16BIT 1

/* Add support for tag numbers > 65536 and fields larger than 65536 bytes. */
/* #define PB_FIELD_32BIT 1 */

/* Disable support for error messages in order to save some code space. */
#define PB_NO_ERRMSG 1

/* Disable support for custom streams (support only memory buffers). */
/* #define PB_BUFFER_ONLY 1 */

/* Switch back to the old-style callback function signature.
 * This was the default until nanopb-0.2.1. */
/* #define PB_OLD_CALLBACK_STYLE */


/* Don't encode scalar arrays as packed. This is only to be used when
 * the decoder on the receiving side cannot process packed scalar arrays.
 * Such example is older protobuf.js. */
/* #define PB_ENCODE_ARRAYS_UNPACKED 1 */

/******************************************************************
 * You usually don't need to change anything below this line.     *
 * Feel free to look around and use the defined macros, though.   *
 ******************************************************************/


/* Version of the nanopb library. Just in case you want to check it in
 * your own program. */
#define NANOPB_VERSION nanopb-0.3.9.8

/* Include all the system headers needed by nanopb. You will need the
 * definitions of the following:
 * - strlen, memcpy, memset functions
 * - [u]int_least8_t, uint_fast8_t, [u]int_least16_t, [u]int32_t, [u]int64_t
 * - size_t
 * - bool
 *
 * If you don't have the standard header files, you can instead provide
 * a custom header that defines or includes all this. In that case,
 * define PB_SYSTEM_HEADER to the path of this file.
 */
#ifdef PB_SYSTEM_HEADER
#include PB_SYSTEM_HEADER
#else
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#ifdef PB_ENABLE_MALLOC
#include <stdlib.h>
#endif
#endif

/* Macro for defining packed structures (compiler dependent).
 * This just reduces memory requirements, but is not required.
 */
#if defined(PB_NO_PACKED_STRUCTS)
    /* Disable struct packing */
#   define PB_PACKED_STRUCT_START
#   define PB_PACKED_STRUCT_END
#   define pb_packed
#elif defined(__GNUC__) || defined(__clang__)
    /* For GCC and clang */
#   define PB_PACKED_STRUCT_START
#   define PB_PACKED_STRUCT_END
#   define pb_packed __attribute__((packed))
#elif defined(__ICCARM__) || defined(__CC_ARM)
    /* For IAR ARM and Keil MDK-ARM compilers */
#   define PB_PACKED_STRUCT_START _Pragma("pack(push, 1)")
#   define PB_PACKED_STRUCT_END _Pragma("pack(pop)")
#   define pb_packed
#elif defined(_MSC_VER) && (_MSC_VER >= 1500)
    /* For Microsoft Visual C++ */
#   define PB_PACKED_STRUCT_START __pragma(pack(push, 1))
#   define PB_PACKED_STRUCT_END __pragma(pack(pop))
#   define pb_packed
#else
    /* Unknown compiler */
#   define PB_PACKED_STRUCT_START
#   define PB_PACKED_STRUCT_END
#   define pb_packed
#endif

/* Handly macro for suppressing unreferenced-parameter compiler warnings. */
#ifndef PB_UNUSED
#define PB_UNUSED(x) (void)(x)
#endif

/* Compile-time assertion, used for checking compatible compilation options.
 * If this does not work properly on your compiler, use
 * #define PB_NO_STATIC_ASSERT to disable it.
 *
 * But before doing that, check carefully the error message / place where it
 * comes from to see if the error has a real cause. Unfortunately the error
 * message is not always very clear to read, but you can see the reason better
 * in the place where the PB_STATIC_ASSERT macro was called.
 */
#ifndef PB_NO_STATIC_ASSERT
#ifndef PB_STATIC_ASSERT
#define PB_STATIC_ASSERT(COND,MSG) typedef char PB_STATIC_ASSERT_MSG(MSG, __LINE__, __COUNTER__)[(COND)?1:-1];
#define PB_STATIC_ASSERT_MSG(MSG, LINE, COUNTER) PB_STATIC_ASSERT_MSG_(MSG, LINE, COUNTER)
#define PB_STATIC_ASSERT_MSG_(MSG, LINE, COUNTER) pb_static_assertion_##MSG##LINE##COUNTER
#endif
#else
#define PB_STATIC_ASSERT(COND,MSG)
#endif

/* Number of required fields to keep track of. */
#ifndef PB_MAX_REQUIRED_FIELDS
#define PB_MAX_REQUIRED_FIELDS 64
#endif

#if PB_MAX_REQUIRED_FIELDS < 64
#error You should not lower PB_MAX_REQUIRED_FIELDS from the default value (64).
#endif

/* List of possible field types. These are used in the autogenerated code.
 * Least-significant 4 bits tell the scalar type
 * Most-significant 4 bits specify repeated/required/packed etc.
 */

typedef uint_least8_t pb_type_t;

/**** Field data types ****/

/* Numeric types */
#define PB_LTYPE_BOOL    0x00 /* bool */
#define PB_LTYPE_VARINT  0x01 /* int32, int64, enum, bool */
#define PB_LTYPE_UVARINT 0x02 /* uint32, uint64 */
#define PB_LTYPE_SVARINT 0x03 /* sint32, sint64 */
#define PB_LTYPE_FIXED32 0x04 /* fixed32, sfixed32, float */
#define PB_LTYPE_FIXED64 0x05 /* fixed64, sfixed64, double */

/* Marker for last packable field type. */
#define PB_LTYPE_LAST_PACKABLE 0x05

/* Byte array with pre-allocated buffer.
 * data_size is the length of the allocated PB_BYTES_ARRAY structure. */
#define PB_LTYPE_BYTES 0x06

/* String with pre-allocated buffer.
 * data_size is the maximum length. */
#define PB_LTYPE_STRING 0x07

/* Submessage
 * submsg_fields is pointer to field descriptions */
#define PB_LTYPE_SUBMESSAGE 0x08

/* Extension pseudo-field
 * The field contains a pointer to pb_extension_t */
#define PB_LTYPE_EXTENSION 0x09

/* Byte array with inline, pre-allocated byffer.
 * data_size is the length of the inline, allocated buffer.
 * This differs from PB_LTYPE_BYTES by defining the element as
 * pb_byte_t[data_size] rather than pb_bytes_array_t. */
#define PB_LTYPE_FIXED_LENGTH_BYTES 0x0A

/* Number of declared LTYPES */
#define PB_LTYPES_COUNT 0x0B
#define PB_LTYPE_MASK 0x0F

/**** Field repetition rules ****/

#define PB_HTYPE_REQUIRED 0x00
#define PB_HTYPE_OPTIONAL 0x10
#define PB_HTYPE_REPEATED 0x20
#define PB_HTYPE_ONEOF    0x30
#define PB_HTYPE_MASK     0x30

/**** Field allocation types ****/
 
#define PB_ATYPE_STATIC   0x00
#define PB_ATYPE_POINTER  0x80
#define PB_ATYPE_CALLBACK 0x40
#define PB_ATYPE_MASK     0xC0

#define PB_ATYPE(x) ((x) & PB_ATYPE_MASK)
#define PB_HTYPE(x) ((x) & PB_HTYPE_MASK)
#define PB_LTYPE(x) ((x) & PB_LTYPE_MASK)

/* Data type used for storing sizes of struct fields
 * and array counts.
 */
#if defined(PB_FIELD_32BIT)
    typedef uint32_t pb_size_t;
    typedef int32_t pb_ssize_t;
#elif defined(PB_FIELD_16BIT)
    typedef uint_least16_t pb_size_t;
    typedef int_least16_t pb_ssize_t;
#else
    typedef uint_least8_t pb_size_t;
    typedef int_least8_t pb_ssize_t;
#endif
#define PB_SIZE_MAX ((pb_size_t)-1)

/* Data type for storing encoded data and other byte streams.
 * This typedef exists to support platforms where uint8_t does not exist.
 * You can regard it as equivalent on uint8_t on other platforms.
 */
typedef uint_least8_t pb_byte_t;

/* This structure is used in auto-generated constants
 * to specify struct fields.
 * You can change field sizes if you need structures
 * larger than 256 bytes or field tags larger than 256.
 * The compiler should complain if your .proto has such
 * structures. Fix that by defining PB_FIELD_16BIT or
 * PB_FIELD_32BIT.
 */
PB_PACKED_STRUCT_START
typedef struct pb_field_s pb_field_t;
struct pb_field_s {
    pb_size_t tag;
    pb_type_t type;
    pb_size_t data_offset; /* Offset of field data, relative to previous field. */
    pb_ssize_t size_offset; /* Offset of array size or has-boolean, relative to data */
    pb_size_t data_size; /* Data size in bytes for a single item */
    pb_size_t array_size; /* Maximum number of entries in array */
    
    /* Field definitions for submessage
     * OR default value for all other non-array, non-callback types
     * If null, then field will zeroed. */
    const void *ptr;
} pb_packed;
PB_PACKED_STRUCT_END

/* Make sure that the standard integer types are of the expected sizes.
 * Otherwise fixed32/fixed64 fields can break.
 *
 * If you get errors here, it probably means that your stdint.h is not
 * correct for your platform.
 */
#ifndef PB_WITHOUT_64BIT
PB_STATIC_ASSERT(sizeof(int64_t) == 2 * sizeof(int32_t), INT64_T_WRONG_SIZE)
PB_STATIC_ASSERT(sizeof(uint64_t) == 2 * sizeof(uint32_t), UINT64_T_WRONG_SIZE)
#endif

/* This structure is used for 'bytes' arrays.
 * It has the number of bytes in the beginning, and after that an array.
 * Note that actual structs used will have a different length of bytes array.
 */
#define PB_BYTES_ARRAY_T(n) struct { pb_size_t size; pb_byte_t bytes[n]; }
#define PB_BYTES_ARRAY_T_ALLOCSIZE(n) ((size_t)n + offsetof(pb_bytes_array_t, bytes))

struct pb_bytes_array_s {
    pb_size_t size;
    pb_byte_t bytes[1];
};
typedef struct pb_bytes_array_s pb_bytes_array_t;

/* This structure is used for giving the callback function.
 * It is stored in the message structure and filled in by the method that
 * calls pb_decode.
 *
 * The decoding callback will be given a limited-length stream
 * If the wire type was string, the length is the length of the string.
 * If the wire type was a varint/fixed32/fixed64, the length is the length
 * of the actual value.
 * The function may be called multiple times (especially for repeated types,
 * but also otherwise if the message happens to contain the field multiple
 * times.)
 *
 * The encoding callback will receive the actual output stream.
 * It should write all the data in one call, including the field tag and
 * wire type. It can write multiple fields.
 *
 * The callback can be null if you want to skip a field.
 */
typedef struct pb_istream_s pb_istream_t;
typedef struct pb_ostream_s pb_ostream_t;
typedef struct pb_callback_s pb_callback_t;
struct pb_callback_s {
#ifdef PB_OLD_CALLBACK_STYLE
    /* Deprecated since nanopb-0.2.1 */
    union {
        bool (*decode)(pb_istream_t *stream, const pb_field_t *field, void *arg);
        bool (*encode)(pb_ostream_t *stream, const pb_field_t *field, const void *arg);
    } funcs;
#else
    /* New function signature, which allows modifying arg contents in callback. */
    union {
        bool (*decode)(pb_istream_t *stream, const pb_field_t *field, void **arg);
        bool (*encode)(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
    } funcs;
#endif    
    
    /* Free arg for use by callback */
    void *arg;
};

/* Wire types. Library user needs these only in encoder callbacks. */
typedef enum {
    PB_WT_VARINT = 0,
    PB_WT_64BIT  = 1,
    PB_WT_STRING = 2,
    PB_WT_32BIT  = 5
} pb_wire_type_t;

/* Structure for defining the handling of unknown/extension fields.
 * Usually the pb_extension_type_t structure is automatically generated,
 * while the pb_extension_t structure is created by the user. However,
 * if you want to catch all unknown fields, you can also create a custom
 * pb_extension_type_t with your own callback.
 */
typedef struct pb_extension_type_s pb_extension_type_t;
typedef struct pb_extension_s pb_extension_t;
struct pb_extension_type_s {
    /* Called for each unknown field in the message.
     * If you handle the field, read off all of its data and return true.
     * If you do not handle the field, do not read anything and return true.
     * If you run into an error, return false.
     * Set to NULL for default handler.
     */
    bool (*decode)(pb_istream_t *stream, pb_extension_t *extension,
                   uint32_t tag, pb_wire_type_t wire_type);
    
    /* Called once after all regular fields have been encoded.
     * If you have something to write, do so and return true.
     * If you do not have anything to write, just return true.
     * If you run into an error, return false.
     * Set to NULL for default handler.
     */
    bool (*encode)(pb_ostream_t *stream, const pb_extension_t *extension);
    
    /* Free field for use by the callback. */
    const void *arg;
};

struct pb_extension_s {
    /* Type describing the extension field. Usually you'll initialize
     * this to a pointer to the automatically generated structure. */
    const pb_extension_type_t *type;
    
    /* Destination for the decoded data. This must match the datatype
     * of the extension field. */
    void *dest;
    
    /* Pointer to the next extension handler, or NULL.
     * If this extension does not match a field, the next handler is
     * automatically called. */
    pb_extension_t *next;

    /* The decoder sets this to true if the extension was found.
     * Ignored for encoding. */
    bool found;
};

/* Memory allocation functions to use. You can define pb_realloc and
 * pb_free to custom functions if you want. */
#ifdef PB_ENABLE_MALLOC
#   ifndef pb_realloc
#       define pb_realloc(ptr, size) realloc(ptr, size)
#   endif
#   ifndef pb_free
#       define pb_free(ptr) free(ptr)
#   endif
#endif

/* This is used to inform about need to regenerate .pb.h/.pb.c files. */
#define PB_PROTO_HEADER_VERSION 30

/* These macros are used to declare pb_field_t's in the constant array. */
/* Size of a structure member, in bytes. */
#define pb_membersize(st, m) (sizeof ((st*)0)->m)
/* Number of entries in an array. */
#define pb_arraysize(st, m) (pb_membersize(st, m) / pb_membersize(st, m[0]))
/* Delta from start of one member to the start of another member. */
#define pb_delta(st, m1, m2) ((int)offsetof(st, m1) - (int)offsetof(st, m2))
/* Marks the end of the field list */
#define PB_LAST_FIELD {0,(pb_type_t) 0,0,0,0,0,0}

/* Macros for filling in the data_offset field */
/* data_offset for first field in a message */
#define PB_DATAOFFSET_FIRST(st, m1, m2) (offsetof(st, m1))
/* data_offset for subsequent fields */
#define PB_DATAOFFSET_OTHER(st, m1, m2) (offsetof(st, m1) - offsetof(st, m2) - pb_membersize(st, m2))
/* data offset for subsequent fields inside an union (oneof) */
#define PB_DATAOFFSET_UNION(st, m1, m2) (PB_SIZE_MAX)
/* Choose first/other based on m1 == m2 (deprecated, remains for backwards compatibility) */
#define PB_DATAOFFSET_CHOOSE(st, m1, m2) (int)(offsetof(st, m1) == offsetof(st, m2) \
                                  ? PB_DATAOFFSET_FIRST(st, m1, m2) \
                                  : PB_DATAOFFSET_OTHER(st, m1, m2))

/* Required fields are the simplest. They just have delta (padding) from
 * previous field end, and the size of the field. Pointer is used for
 * submessages and default values.
 */
#define PB_REQUIRED_STATIC(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_STATIC | PB_HTYPE_REQUIRED | ltype, \
    fd, 0, pb_membersize(st, m), 0, ptr}

/* Optional fields add the delta to the has_ variable. */
#define PB_OPTIONAL_STATIC(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_STATIC | PB_HTYPE_OPTIONAL | ltype, \
    fd, \
    pb_delta(st, has_ ## m, m), \
    pb_membersize(st, m), 0, ptr}

#define PB_SINGULAR_STATIC(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_STATIC | PB_HTYPE_OPTIONAL | ltype, \
    fd, 0, pb_membersize(st, m), 0, ptr}

/* Repeated fields have a _count field and also the maximum number of entries. */
#define PB_REPEATED_STATIC(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_STATIC | PB_HTYPE_REPEATED | ltype, \
    fd, \
    pb_delta(st, m ## _count, m), \
    pb_membersize(st, m[0]), \
    pb_arraysize(st, m), ptr}

/* Allocated fields carry the size of the actual data, not the pointer */
#define PB_REQUIRED_POINTER(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_POINTER | PB_HTYPE_REQUIRED | ltype, \
    fd, 0, pb_membersize(st, m[0]), 0, ptr}

/* Optional fields don't need a has_ variable, as information would be redundant */
#define PB_OPTIONAL_POINTER(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_POINTER | PB_HTYPE_OPTIONAL | ltype, \
    fd, 0, pb_membersize(st, m[0]), 0, ptr}

/* Same as optional fields*/
#define PB_SINGULAR_POINTER(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_POINTER | PB_HTYPE_OPTIONAL | ltype, \
    fd, 0, pb_membersize(st, m[0]), 0, ptr}

/* Repeated fields have a _count field and a pointer to array of pointers */
#define PB_REPEATED_POINTER(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_POINTER | PB_HTYPE_REPEATED | ltype, \
    fd, pb_delta(st, m ## _count, m), \
    pb_membersize(st, m[0]), 0, ptr}

/* Callbacks are much like required fields except with special datatype. */
#define PB_REQUIRED_CALLBACK(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_CALLBACK | PB_HTYPE_REQUIRED | ltype, \
    fd, 0, pb_membersize(st, m), 0, ptr}

#define PB_OPTIONAL_CALLBACK(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_CALLBACK | PB_HTYPE_OPTIONAL | ltype, \
    fd, 0, pb_membersize(st, m), 0, ptr}

#define PB_SINGULAR_CALLBACK(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_CALLBACK | PB_HTYPE_OPTIONAL | ltype, \
    fd, 0, pb_membersize(st, m), 0, ptr}
    
#define PB_REPEATED_CALLBACK(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_CALLBACK | PB_HTYPE_REPEATED | ltype, \
    fd, 0, pb_membersize(st, m), 0, ptr}

/* Optional extensions don't have the has_ field, as that would be redundant.
 * Furthermore, the combination of OPTIONAL without has_ field is used
 * for indicating proto3 style fields. Extensions exist in proto2 mode only,
 * so they should be encoded according to proto2 rules. To avoid the conflict,
 * extensions are marked as REQUIRED instead.
 */
#define PB_OPTEXT_STATIC(tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_STATIC | PB_HTYPE_REQUIRED | ltype, \
    0, \
    0, \
    pb_membersize(st, m), 0, ptr}

#define PB_OPTEXT_POINTER(tag, st, m, fd, ltype, ptr) \
    PB_OPTIONAL_POINTER(tag, st, m, fd, ltype, ptr)

#define PB_OPTEXT_CALLBACK(tag, st, m, fd, ltype, ptr) \
    PB_OPTIONAL_CALLBACK(tag, st, m, fd, ltype, ptr)

/* The mapping from protobuf types to LTYPEs is done using these macros. */
#define PB_LTYPE_MAP_BOOL               PB_LTYPE_BOOL
#define PB_LTYPE_MAP_BYTES              PB_LTYPE_BYTES
#define PB_LTYPE_MAP_DOUBLE             PB_LTYPE_FIXED64
#define PB_LTYPE_MAP_ENUM               PB_LTYPE_VARINT
#define PB_LTYPE_MAP_UENUM              PB_LTYPE_UVARINT
#define PB_LTYPE_MAP_FIXED32            PB_LTYPE_FIXED32
#define PB_LTYPE_MAP_FIXED64            PB_LTYPE_FIXED64
#define PB_LTYPE_MAP_FLOAT              PB_LTYPE_FIXED32
#define PB_LTYPE_MAP_INT32              PB_LTYPE_VARINT
#define PB_LTYPE_MAP_INT64              PB_LTYPE_VARINT
#define PB_LTYPE_MAP_MESSAGE            PB_LTYPE_SUBMESSAGE
#define PB_LTYPE_MAP_SFIXED32           PB_LTYPE_FIXED32
#define PB_LTYPE_MAP_SFIXED64           PB_LTYPE_FIXED64
#define PB_LTYPE_MAP_SINT32             PB_LTYPE_SVARINT
#define PB_LTYPE_MAP_SINT64             PB_LTYPE_SVARINT
#define PB_LTYPE_MAP_STRING             PB_LTYPE_STRING
#define PB_LTYPE_MAP_UINT32             PB_LTYPE_UVARINT
#define PB_LTYPE_MAP_UINT64             PB_LTYPE_UVARINT
#define PB_LTYPE_MAP_EXTENSION          PB_LTYPE_EXTENSION
#define PB_LTYPE_MAP_FIXED_LENGTH_BYTES PB_LTYPE_FIXED_LENGTH_BYTES

/* This is the actual macro used in field descriptions.
 * It takes these arguments:
 * - Field tag number
 * - Field type:   BOOL, BYTES, DOUBLE, ENUM, UENUM, FIXED32, FIXED64,
 *                 FLOAT, INT32, INT64, MESSAGE, SFIXED32, SFIXED64
 *                 SINT32, SINT64, STRING, UINT32, UINT64 or EXTENSION
 * - Field rules:  REQUIRED, OPTIONAL or REPEATED
 * - Allocation:   STATIC, CALLBACK or POINTER
 * - Placement: FIRST or OTHER, depending on if this is the first field in structure.
 * - Message name
 * - Field name
 * - Previous field name (or field name again for first field)
 * - Pointer to default value or submsg fields.
 */

#define PB_FIELD(tag, type, rules, allocation, placement, message, field, prevfield, ptr) \
        PB_ ## rules ## _ ## allocation(tag, message, field, \
        PB_DATAOFFSET_ ## placement(message, field, prevfield), \
        PB_LTYPE_MAP_ ## type, ptr)

/* Field description for repeated static fixed count fields.*/
#define PB_REPEATED_FIXED_COUNT(tag, type, placement, message, field, prevfield, ptr) \
    {tag, PB_ATYPE_STATIC | PB_HTYPE_REPEATED | PB_LTYPE_MAP_ ## type, \
    PB_DATAOFFSET_ ## placement(message, field, prevfield), \
    0, \
    pb_membersize(message, field[0]), \
    pb_arraysize(message, field), ptr}

/* Field description for oneof fields. This requires taking into account the
 * union name also, that's why a separate set of macros is needed.
 */
#define PB_ONEOF_STATIC(u, tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_STATIC | PB_HTYPE_ONEOF | ltype, \
    fd, pb_delta(st, which_ ## u, u.m), \
    pb_membersize(st, u.m), 0, ptr}

#define PB_ONEOF_POINTER(u, tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_POINTER | PB_HTYPE_ONEOF | ltype, \
    fd, pb_delta(st, which_ ## u, u.m), \
    pb_membersize(st, u.m[0]), 0, ptr}

#define PB_ONEOF_FIELD(union_name, tag, type, rules, allocation, placement, message, field, prevfield, ptr) \
        PB_ONEOF_ ## allocation(union_name, tag, message, field, \
        PB_DATAOFFSET_ ## placement(message, union_name.field, prevfield), \
        PB_LTYPE_MAP_ ## type, ptr)

#define PB_ANONYMOUS_ONEOF_STATIC(u, tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_STATIC | PB_HTYPE_ONEOF | ltype, \
    fd, pb_delta(st, which_ ## u, m), \
    pb_membersize(st, m), 0, ptr}

#define PB_ANONYMOUS_ONEOF_POINTER(u, tag, st, m, fd, ltype, ptr) \
    {tag, PB_ATYPE_POINTER | PB_HTYPE_ONEOF | ltype, \
    fd, pb_delta(st, which_ ## u, m), \
    pb_membersize(st, m[0]), 0, ptr}

#define PB_ANONYMOUS_ONEOF_FIELD(union_name, tag, type, rules, allocation, placement, message, field, prevfield, ptr) \
        PB_ANONYMOUS_ONEOF_ ## allocation(union_name, tag, message, field, \
        PB_DATAOFFSET_ ## placement(message, field, prevfield), \
        PB_LTYPE_MAP_ ## type, ptr)

/* These macros are used for giving out error messages.
 * They are mostly a debugging aid; the main error information
 * is the true/false return value from functions.
 * Some code space can be saved by disabling the error
 * messages if not used.
 *
 * PB_SET_ERROR() sets the error message if none has been set yet.
 *                msg must be a constant string literal.
 * PB_GET_ERROR() always returns a pointer to a string.
 * PB_RETURN_ERROR() sets the error and returns false from current
 *                   function.
 */
#ifdef PB_NO_ERRMSG
#define PB_SET_ERROR(stream, msg) PB_UNUSED(stream)
#define PB_GET_ERROR(stream) "(errmsg disabled)"
#else
#define PB_SET_ERROR(stream, msg) (stream->errmsg = (stream)->errmsg ? (stream)->errmsg : (msg))
#define PB_GET_ERROR(stream) ((stream)->errmsg ? (stream)->errmsg : "(none)")
#endif

#define PB_RETURN_ERROR(stream, msg) return PB_SET_ERROR(stream, msg), false

// Custom PB defines, to support PROGMEM reading
// Calculate the offset into the struct
#define STRUCT_VAL_byte(MemOffset, FieldName) (pgm_read_byte(&(MemOffset->FieldName)))
#define STRUCT_VAL_word(MemOffset, FieldName) (pgm_read_word(&(MemOffset->FieldName)))
#define STRUCT_VAL_dword(MemOffset, FieldName) (pgm_read_dword(&(MemOffset->FieldName)))

#endif
