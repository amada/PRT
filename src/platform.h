#pragma once

#ifdef _MSC_VER
#define R_MSVC
#endif

#ifdef _M_ARM64 // Pre-defined macro in MSVC for Arm64 architecture
#define R_NEON
#define R_ARM64
#elif defined(__ARM_NEON)
#define R_NEON
#ifdef __aarch64__
#define R_ARM64
#endif
#else // __ARM_NEON
//#define R_AVX4L
#define R_AVX8L
#endif // _ARM_NEON
