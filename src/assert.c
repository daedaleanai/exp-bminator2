// Implemnenatation of assert_func so assert.h and assert() work

#include "cortex_m4.h"
#include "tprintf.h"

#define printf tprintf

void __assert_func (const char * file, int line, const char *func, const char *expr) {
    printf("\e[K\n%s:%d (%s) assertion failed: %s\e[K\n", file, line, func,expr);
    for(;;)
        __BKPT();
}
