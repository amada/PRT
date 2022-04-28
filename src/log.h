#pragma once

namespace prt
{

enum class LogLevel
{
    kInfo,
    kVerbose,
    kError
};

void logPrintf(LogLevel level, const char* format...);

} // namespace prt
