//
//	xphysics rigid body dynamics engine
//
//	MIT License
//	Copyright (C) 2026 Alan Moczulski
//	
//	Permission is hereby granted, free of charge, to any person obtaining a copy
//	of this software and associated documentation files(the "Software"), to deal
//	in the Software without restriction, including without limitation the rights
//	to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
//	copies of the Software, and to permit persons to whom the Software is
//	furnished to do so, subject to the following conditions :
//	
//	The above copyright notice and this permission notice shall be included in all
//	copies or substantial portions of the Software.
//	
//	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
//	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//	SOFTWARE.
//

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
#ifndef XP_USE_DOUBLE_PRECISION
#define XP_USE_DOUBLE_PRECISION 1
#endif

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

#define INVALID_ID (id)(-1)

// memory management
XP_EXTERN_C XP_API usize xp_get_persistent_memory_requirements(u32 num_convex_hull_verts, u32 num_bodies);
XP_EXTERN_C XP_API usize xp_get_transient_memory_requirements(u32 num_contacts, u32 num_bodies);

// main functions
XP_EXTERN_C XP_API bool xp_init(const memory_provider* provider, u32 convex_hulls_verts_budget, u32 bodies_budget);
XP_EXTERN_C XP_API void xp_uninit();
XP_EXTERN_C XP_API void xp_step(second dt);

// rigid body functions
XP_EXTERN_C XP_API id xp_create_fixed_body();
XP_EXTERN_C XP_API id xp_create_dynamic_body(kilogram mass); // todo: provide inertia type enum? inertia must be known for proper dynamics
XP_EXTERN_C XP_API void xp_destroy_body(id body_id);
XP_EXTERN_C XP_API void xp_attach_shape(id body_id, id shape_id);
XP_EXTERN_C XP_API void xp_get_body_position(id body_id, real out_position[3]);
XP_EXTERN_C XP_API void xp_set_body_position(id body_id, const real position[3]);
// todo: push body

// shape functions
XP_EXTERN_C XP_API id xp_create_convex_hull(const real* vertex_positions, u32 vertex_count);
// todo: terrain/heightfield
