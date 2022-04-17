#pragma once

//#define PRT_ENABLE_STATS 1

#define PRT_ASSERT(x) { if (!(x)) { printf("Assertion failed at line %d in %s\n", __LINE__, __FILE__); __builtin_trap(); }}
