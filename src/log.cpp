#include <stdarg.h>
#include <stdio.h>

#include "log.h"

namespace prt
{

void logPrintf(LogLevel level, const char* format...)
{
    switch (level) {
    case LogLevel::kVerbose: break;
    case LogLevel::kError: printf("ERROR: "); break;
    default:
        __builtin_trap();
    };

    va_list args;
    va_start(args, format);
    vfprintf(stdout, format, args);
    va_end(args);
}

} // namespace prt
