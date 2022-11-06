#pragma once

#include "platform.h"

//#define PRT_ENABLE_STATS 1

#ifdef R_MSVC
#define PRT_DEBUGBREAK() __debugbreak()
#define PRT_ASSERT(x) { if (!(x)) { printf("Assertion failed at line %d in %s\n", __LINE__, __FILE__); __debugbreak(); }}
#else
#define PRT_DEBUGBREAK() __builtin_trap()
#define PRT_ASSERT(x) { if (!(x)) { printf("Assertion failed at line %d in %s\n", __LINE__, __FILE__); __builtin_trap(); }}
#endif