//
//	xphysics math
//
//	(C) 2026 Alan Moczulski
//
//	Notes:
//	- x64 calling conventions can pass __m128 and __m256 in registers natively if the types are small enough,
//	but it's still worth checking the disassembly to make sure the compiler hasn't done anything crazy
//	- row major storage (could be interesting to test column major too)
//

#pragma once
#include "xp.hxx"
#include <math.h>

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

namespace xp
{
	typedef union
	{
		struct { real x, y, z, w; };
		real data[4];
		vreal4simd simd;
	} __declspec(align(32)) vreal4; // important: must be aligned to 32 bytes for AVX!

	// vector addition
	inline vreal4 vadd(const vreal4 a, const vreal4 b)
	{
		vreal4 result;
		result.simd = _mm256_add_pd(a.simd, b.simd);
		return result;
	}

	// vector subtraction
	inline vreal4 vsub(const vreal4 a, const vreal4 b)
	{
		vreal4 result;
		result.simd = _mm256_sub_pd(a.simd, b.simd);
		return result;
	}

	// vector dot product
	inline real vdot(const vreal4 a, const vreal4 b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
	}

	// vector cross product
	inline vreal4 vcross(const vreal4 a, const vreal4 b)
	{
		vreal4 result;
		result.x = a.y * b.z - a.z * b.y;
		result.y = a.z * b.x - a.x * b.z;
		result.z = a.x * b.y - a.y * b.x;
		result.w = 0.0;
		return result;
	}

	// quaternion normalization
	inline vreal4 vnormalize(const vreal4 v)
	{
		const real dp = vdot(v, v);

		// todo: simd optimization possible
		const real len = 1.0 / sqrt(dp);
		
		vreal4 result;
		result.x = v.x * len;
		result.y = v.y * len;
		result.z = v.z * len;
		result.w = v.w * len;
		return result;
	}

	// quaternion
	typedef union
	{
		struct { real i, j, k, r; };
		real data[4];
		vreal4simd simd;
	} __declspec(align(32)) qreal; // important: must be aligned to 32 bytes for AVX!

	// quaternion dot product
	inline real qdot(const qreal a, const qreal b)
	{
		return a.i * b.i + a.j * b.j + a.k * b.k + a.r * b.r;
	}

	// quaternion normalization
	inline qreal qnormalize(const qreal q)
	{
		const real dp = qdot(q, q);
		if (dp <= 1.0e-20)
		{
			const qreal default_q = { 0.0, 0.0, 0.0, 1.0 };
			return default_q;
		}

		if (dp >= 1.0 && dp - 1.0 <= 2.0e-10)
			return q;

		if (dp < 1.0 && 1.0 - dp <= 2.0e-10)
			return q;

		// todo: simd optimization possible
		const real len = 1.0 / sqrt(dp);
		
		qreal result;
		result.i = q.i * len;
		result.j = q.j * len;
		result.k = q.k * len;
		result.r = q.r * len;
		
		return result;
	}

	// quaternion conjugate
	inline qreal qconjugate(const qreal q)
	{
		const qreal result = { -q.i, -q.j, -q.k, q.r };
		return result;
	}

	// quaternion from angles
	inline qreal qangles(real x, real y, real z)
	{
		const real hx = x * 0.5f;
		const real hy = y * 0.5f;
		const real hz = z * 0.5f;

		// todo: can we use a single SSE intrinsic here for sine and cosine computation?
		const real shx = sin(hx);
		const real chx = cos(hx);
		const real shy = sin(hy);
		const real chy = cos(hy);
		const real shz = sin(hz);
		const real chz = cos(hz);

		const real sc = shx * chy;
		const real cs = chx * shy;
		const real cc = chx * chy;
		const real ss = shx * shy;

		qreal result;
		result.i = cs * chz + sc * shz;
		result.j = sc * chz - cs * shz;
		result.k = cc * shz - ss * chz;
		result.r = ss * shz + cc * chz;
		return result;
	}

	typedef union
	{
		real data[12];
		vreal4 rows[3];
	} mreal3x4; // useful for affine transformations

	// matrix-vector multiplication
	inline vreal4 m3x4vmul(const mreal3x4* m, const vreal4 v)
	{
		vreal4 result;
		result.x = vdot(m->rows[0], v);
		result.y = vdot(m->rows[1], v);
		result.z = vdot(m->rows[2], v);
		result.w = v.w; // preserve w (1.0 for points, 0.0 for vectors)
		return result;
	}

	typedef union
	{
		real data[16];
		vreal4 rows[4];
	} mreal4x4;

	// matrix-vector multiplication
	inline vreal4 m4x4vmul(const mreal4x4* m, const vreal4 v)
	{
		vreal4 result;
		result.x = vdot(m->rows[0], v);
		result.y = vdot(m->rows[1], v);
		result.z = vdot(m->rows[2], v);
		result.w = vdot(m->rows[3], v);
		return result;
	}
}
