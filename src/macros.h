#ifndef MACROS_H
#define MACROS_H

#define FORCE_INLINE static inline __attribute__((__always_inline__))

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#define __CREATE_REG(prefix, index, postfix) prefix ## index ## postfix
#define _CREATE_REG(prefix, index, postfix) __CREATE_REG(prefix, index, postfix)

#endif//MACROS_H
