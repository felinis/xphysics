#pragma once

// architecture bitness
#if defined(__amd64__) || defined(__x86_64__) || defined(_M_AMD64) || defined(_M_X64)
#define XP_ARCH_BITNESS_64 1
#elif defined(__i386__) || defined(_M_IX86)
#define XP_ARCH_BITNESS_32 1
#else
#error "Unsupported architecture"
#endif

#ifndef NDEBUG
#define XP_BREAK(x) { volatile int *p = 0; *p = 0; } // debugger break
#else
#define XP_BREAK(x)
#endif

#if defined(_WIN32) || defined(__CYGWIN__)
	#if XP_EXPORT
		#define XP_API __declspec(dllexport)
	#else
		#define XP_API __declspec(dllimport)
	#endif
#endif

#ifdef __cplusplus
#define XP_EXTERN_C extern "C"
#else
#define XP_EXTERN_C extern
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
typedef usize id;

typedef float f32;
typedef double f64;

#if XP_USE_DOUBLE_PRECISION
typedef f64 real;
#else
typedef f32 real;
#endif

typedef real kilogram;
typedef real second;

typedef struct
{
	u8* persistent_memory;
	usize persistent_size;

	u8* transient_memory;
	usize transient_size;
} memory_provider;

constexpr id INVALID_ID = (id)-1;

XP_EXTERN_C XP_API usize xp_get_memory_requirements(u32 num_convex_hull_verts, u32 num_contacts, u32 num_bodies);
XP_EXTERN_C XP_API bool xp_init(const memory_provider* provider, u32 convex_hulls_verts_budget, u32 bodies_budget);
XP_EXTERN_C XP_API void xp_uninit();
XP_EXTERN_C XP_API void xp_step(second dt);
XP_EXTERN_C XP_API id xp_create_fixed_body();
XP_EXTERN_C XP_API id xp_create_dynamic_body(kilogram mass); // todo: inertia type enum?
XP_EXTERN_C XP_API void xp_destroy_body(id body_id);
XP_EXTERN_C XP_API id xp_create_convex_hull(const real* vertex_positions, u32 vertex_count);
XP_EXTERN_C XP_API void xp_attach_shape(id body_id, id shape_id);
XP_EXTERN_C XP_API void xp_get_body_position(id body_id, real out_position[3]);
XP_EXTERN_C XP_API void xp_set_body_position(id body_id, const real position[3]);
