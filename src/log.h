#pragma once

namespace prt
{

enum class LogLevel
{
    kVerbose,
    kError
};

void logPrintf(LogLevel level, const char* format...);

} // namespace prt
