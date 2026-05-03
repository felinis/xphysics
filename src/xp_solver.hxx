#pragma once
#include "xp_internal.hxx"
#include "xp_math.hxx"

struct XPContactManifold
{
	id body_a, body_b;
	real penetration_depth;
	real accumulated_impulse; // for warm starting the solver
	vreal4 pos;
	vreal4 normal; // from b to a
};

void XPSolveContacts(
	u32 num_manifolds,
	XPContactManifold* manifolds,
	vreal4* positions,
	vreal4* linear_velocities,
	qreal* angular_velocities,
	real* inv_masses,
	vreal4* inv_inertias,
	second dt
);
