#include "../src/xp_math.h"

int main()
{
	// test 1: addition
	const vreal4 a = { -200.0, 1050.0, 0.0, 5.0 };
	const vreal4 b = { 200.0, -50.0, 1.0, -710.0 };
	const vreal4 c = vadd(a, b);
	if (c.x != 0.0 || c.y != 1000.0 || c.z != 1.0 || c.w != -705.0)
		return 1;

	// test 2: subtraction
	const vreal4 d = vsub(a, b);
	if (d.x != -400.0 || d.y != 1100.0 || d.z != -1.0 || d.w != 715.0)
		return 2;

	return 0;
}
