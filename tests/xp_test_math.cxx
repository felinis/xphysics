#include "../src/xp_math.hxx"
using namespace xp;

int main()
{
	// vector addition
	const vreal4 a = { -200.0, 1050.0, 0.0, 5.0 };
	const vreal4 b = { 200.0, -50.0, 1.0, -710.0 };
	const vreal4 c = vadd(a, b);
	if (c.x != 0.0 || c.y != 1000.0 || c.z != 1.0 || c.w != -705.0)
		return 1;

	// vector subtraction
	const vreal4 d = vsub(a, b);
	if (d.x != -400.0 || d.y != 1100.0 || d.z != -1.0 || d.w != 715.0)
		return 2;

	// vector dot product
	real dot = vdot(a, b);
	if (dot != -96050.0) // (-200*200) + (1050*-50) + 0 + (5*-710) = -40000 - 52500 - 3550 = -96050
		return 3;

	// vector cross product
	const vreal4 e = { 1.0, 0.0, 0.0, 0.0 };
	const vreal4 f = { 0.0, 1.0, 0.0, 0.0 };
	const vreal4 g = vcross(e, f);
	if (g.x != 0.0 || g.y != 0.0 || g.z != 1.0 || g.w != 0.0)
		return 4;

	// vector normalization
	const vreal4 v1 = { 1.0, 0.0, 0.0, 1.0 };
	const vreal4 v1_norm = vnormalize(v1); // magnitude is sqrt(2), so each should be 1/sqrt(2)
	const real inv_sqrt2 = 1.0 / sqrt(2.0);
	if (fabs(v1_norm.x - inv_sqrt2) > 1e-6 || fabs(v1_norm.w - inv_sqrt2) > 1e-6)
		return 5;

	// quaternion
	const qreal q1 = { 1.0, 0.0, 0.0, 1.0 };
	const qreal q1_norm = qnormalize(q1); // magnitude is sqrt(2), so each should be 1/sqrt(2)
	if (fabs(q1_norm.i - inv_sqrt2) > 1e-6 || fabs(q1_norm.r - inv_sqrt2) > 1e-6)
		return 6;
		
	const qreal q2 = { 0.0, 1.0, 0.0, 0.0 };
	if (qdot(q1, q2) != 0.0)
		return 7;
		
	const qreal q3 = qconjugate(q1);
	if (q3.i != -1.0 || q3.j != 0.0 || q3.k != 0.0 || q3.r != 1.0)
		return 8;
		
	const qreal q4 = qangles(0.0, 0.0, 0.0); // identity quaternion
	if (q4.i != 0.0 || q4.j != 0.0 || q4.k != 0.0 || q4.r != 1.0)
		return 9;

	// test 6: matrix multiplication
	const mreal4x4 h = { 1.0, 0.0, 0.0, 10.0, 0.0, 1.0, 0.0, 20.0, 0.0, 0.0, 1.0, 30.0, 0.0, 0.0, 0.0, 1.0 };
	const vreal4 i = { 5.0, 5.0, 5.0, 1.0 };
	const vreal4 j = m4x4vmul(&h, i);
	if (j.x != 15.0 || j.y != 25.0 || j.z != 35.0 || j.w != 1.0)
		return 10;

	return 0;
}
