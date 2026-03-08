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
typedef void* xp_shape;
typedef void* xp_rigid_body;

// MAIN API
typedef struct
{
	void* persistent_memory; // memory block that will be used for persistent data (e.g. convex hulls, rigid bodies)
	usize persistent_memory_size;
	void* transient_memory; // memory block that will be used for transient data (e.g. solving contacts)
	usize transient_memory_size;
	
	usize max_convex_hulls;
	usize max_rigid_bodies;
} xp_context_create_info;
XP_API xp_context xp_init(const xp_context_create_info* info);
XP_API void xp_shutdown(xp_context context);
XP_API void xp_update(xp_context context, real dt);

// SHAPE API
XP_API xp_shape xp_create_convex_hull_shape(xp_context context, real* vertex_positions, usize vertex_count);
XP_API void xp_destroy_shape(xp_context context, xp_shape shape);

// RIGID BODY API
XP_API xp_rigid_body xp_create_rigid_body(xp_context context, xp_shape shape);
XP_API void xp_destroy_rigid_body(xp_context context, xp_rigid_body body);

