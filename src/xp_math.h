//
//	xphysics math
//
//	(C) 2026 Alan Moczulski
//
//	Notes:
//	- x64 calling conventions can pass __m128 and __m256 in registers natively if the types are small enough,
//	but it's still worth checking the disassembly to make sure the compiler hasn't done anything crazy
//

#pragma once
#include "xp.h"

#if XP_USE_DOUBLE_PRECISION
	#if defined(__AVX__)
		#include <immintrin.h>
		typedef __m256d vreal4simd; // we use 4 doubles
	#else
		#error "AVX instruction set is required for double precision math"
	#endif
#else
	#error "Only 64-bit floating point numbers are supported currently"
#endif

typedef union
{
	struct { real x, y, z, w; };
	real data[4];
	vreal4simd simd;
} __declspec(align(32)) vreal4; // important: must be aligned to 32 bytes for AVX!

inline vreal4 vadd(const vreal4 a, const vreal4 b)
{
	vreal4 result;
	result.simd = _mm256_add_pd(a.simd, b.simd);
	return result;
}

inline vreal4 vsub(const vreal4 a, const vreal4 b)
{
	vreal4 result;
	result.simd = _mm256_sub_pd(a.simd, b.simd);
	return result;
}

inline real vdot(const vreal4 a, const vreal4 b)
{

}

inline vreal4 vcross(const vreal4 a, const vreal4 b)
{

}
