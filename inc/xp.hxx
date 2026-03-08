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

#ifdef __cplusplus
#define XP_EXTERN_C extern "C"
#else
#define XP_EXTERN_C extern
#endif

// xphysics configuration
// feel free to change the following macros to match your needs
#define XP_USE_DOUBLE_PRECISION 1

namespace xp
{
	// type definitions
	typedef char s8;
	typedef unsigned char u8;
	typedef short s16;
	typedef unsigned short u16;
	typedef int s32;
	typedef unsigned int u32;
	typedef u32 id;
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

	typedef real kilogram;
	typedef real second;

	struct memory_provider
	{
		u8* persistent_memory;
		usize persistent_size;

		u8* transient_memory;
		usize transient_size;
	};

	struct instance
	{
		virtual void step(second dt) = 0;

		virtual id create_fixed_body() = 0;
		virtual id create_dynamic_body(kilogram mass) = 0; // todo: inertia type enum?
		virtual void destroy_body(id body_id) = 0;

		virtual id create_convex_shape(const real* vertex_positions, u32 vertex_count) = 0;
		virtual void attach_shape(id body_id, id shape_id) = 0;

		virtual void get_body_position(id body_id, real out_position[3]) const = 0;
		virtual void set_body_position(id body_id, const real position[3]) = 0;
	};
}

XP_EXTERN_C XP_API xp::instance* xp_create_instance(const xp::memory_provider& provider);
