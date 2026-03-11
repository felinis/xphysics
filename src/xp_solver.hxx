#pragma once
#include "xp_internal.hxx"
#include "xp_math.hxx"

struct xp_contact_manifold
{
    id body_a, body_b;

    real penetration_depth;
    real accumulated_impulse; // for warm starting the solver
    
    vreal4 pos;
    vreal4 normal; // from b to a
};

void xp_solve_contacts(
	u32 num_manifolds,
	xp_contact_manifold* manifolds,
	vreal4* positions,
	vreal4* linear_velocities,
	vreal4* angular_velocities,
	real* inv_masses,
	real* inv_inertias,
	second dt
);
