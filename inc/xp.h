#pragma once

// architecture bitness
#if defined(__amd64__) || defined(__x86_64__) || defined(_M_AMD64) || defined(_M_X64)
#define XP_ARCH_BITNESS_64 1
#elif defined(__i386__) || defined(_M_IX86)
#define XP_ARCH_BITNESS_32 1
#else
#error "Unsupported architecture"
#endif

#if defined(_WIN32) || defined(__CYGWIN__)
	#if XP_EXPORT
		#define XP_API __declspec(dllexport)
	#else
		#define XP_API __declspec(dllimport)
	#endif
#endif


// xphysics configuration
// feel free to change the following macros to match your needs
#define XP_USE_DOUBLE_PRECISION 1

// type definitions
typedef char s8;
typedef unsigned char u8;
typedef short s16;
typedef unsigned short u16;
typedef int s32;
typedef unsigned int u32;
typedef long long s64;
typedef unsigned long long u64;

// we define usize and ssize depending on the architecture bitness
#if XP_ARCH_BITNESS_64
typedef u64 usize;
typedef s64 ssize;
#elif XP_ARCH_BITNESS_32
typedef u32 usize;
typedef s32 ssize;
#endif

typedef float f32;
typedef double f64;

#if XP_USE_DOUBLE_PRECISION
typedef f64 real;
#else
typedef f32 real;
#endif

typedef void* xp_context;

XP_API xp_context xp_init(void* mem, usize size);
XP_API void xp_shutdown(xp_context context);
XP_API void xp_update(xp_context context, real dt);
