#pragma once
#include "xp_math.hxx"

// unary minus
inline vreal4 operator-(const vreal4 a)
{
	vreal4 result;
	result.simd = _mm256_sub_pd(_mm256_setzero_pd(), a.simd);
	return result;
}

// binary plus
inline vreal4 operator+(const vreal4 a, const vreal4 b)
{
	return vadd(a, b);
}

// binary minus
inline vreal4 operator-(const vreal4 a, const vreal4 b)
{
	return vsub(a, b);
}

// vector * scalar
inline vreal4 operator*(const vreal4 a, const real b)
{
	vreal4 result;
	result.simd = _mm256_mul_pd(a.simd, _mm256_set1_pd(b));
	return result;
}

// scalar * vector
inline vreal4 operator*(const real a, const vreal4 b)
{
	vreal4 result;
	result.simd = _mm256_mul_pd(_mm256_set1_pd(a), b.simd);
	return result;
}

// vector / scalar
inline vreal4 operator/(const vreal4 a, const real b)
{
	vreal4 result;
	result.simd = _mm256_div_pd(a.simd, _mm256_set1_pd(b));
	return result;
}

// compound assignments
inline vreal4& operator+=(vreal4& a, const vreal4 b)
{
	a = a + b;
	return a;
}

inline vreal4& operator-=(vreal4& a, const vreal4 b)
{
	a = a - b;
	return a;
}

inline vreal4& operator*=(vreal4& a, const real b)
{
	a = a * b;
	return a;
}

inline vreal4& operator/=(vreal4& a, const real b)
{
	a = a / b;
	return a;
}
