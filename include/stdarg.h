#ifndef __STDARG_H
#define __STDARG_H

// m68k (gcc ABI): every argument is on the stack, each in a slot whose size
// is rounded up to a multiple of 4 bytes. A char/short parameter sits at the
// END of its 4-byte slot (big-endian), so the next argument starts right
// after the last named parameter: &last + sizeof(last).
typedef char *va_list;

#define __va_slot(t) ((sizeof(t) + 3) & ~3)

#define va_start(ap, last) ((ap) = (char *)&(last) + sizeof(last))
#define va_end(ap) ((void)0)
#define va_arg(ap, t) (*(t *)(((ap) += __va_slot(t)) - __va_slot(t)))
#define va_copy(dest, src) ((dest) = (src))

#define __GNUC_VA_LIST 1
typedef va_list __gnuc_va_list;

#endif
