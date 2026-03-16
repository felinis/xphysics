//
//	xphysics rigid body dynamics engine
//	(C) 2026 Alan Moczulski
//

#include "xp_solver.hxx"
#include "xp_math_operators.hxx"

void xp_solve_contacts(
	u32 num_manifolds,
	xp_contact_manifold* manifolds,
	vreal4* positions,
	vreal4* linear_velocities,
	vreal4* angular_velocities,
	real* inv_masses,
	real* inv_inertias,
	second dt
)
{
	const u32 NUM_ITERATIONS = 10;
	const real BAUMGARTE_BETA = 0.2; // factor for positional error correction
	const real ALLOWED_PENETRATION = 0.001; // to prevent jitter

	// pgs iterative solver
	for (u32 it = 0; it < NUM_ITERATIONS; ++it)
	{
		// todo: can we parallelize this?
		for (u32 i = 0; i < num_manifolds; ++i)
		{
			xp_contact_manifold& c = manifolds[i];

			const vreal4 n = c.normal;
			const vreal4 p = c.pos;
			const vreal4 posA = positions[c.body_a];
			const vreal4 posB = positions[c.body_b];
			vreal4& vA = linear_velocities[c.body_a];
			vreal4& vB = linear_velocities[c.body_b];
			vreal4& wA = angular_velocities[c.body_a];
			vreal4& wB = angular_velocities[c.body_b];
			const real inv_mass_a = inv_masses[c.body_a];
			const real inv_mass_b = inv_masses[c.body_b];
			const real inv_inertia_a = inv_inertias[c.body_a];
			const real inv_inertia_b = inv_inertias[c.body_b];

			// calculate vectors from center of mass to contact point
			const vreal4 rA = p - posA;
			const vreal4 rB = p - posB;

			// calculate relative velocity at the contact point
			const vreal4 vA_point = vA + vcross(wA, rA);
			const vreal4 vB_point = vB + vcross(wB, rB);
			const vreal4 v_rel = vA_point - vB_point;

			// velocity along the normal
			real v_rel_n = vdot(v_rel, n);

			// effective masses
			const vreal4 rA_cross_n = vcross(rA, n);
			const vreal4 rB_cross_n = vcross(rB, n);
			
			const real termA = inv_mass_a + vdot(vcross(rA_cross_n, rA), n) * inv_inertia_a;
			const real termB = inv_mass_b + vdot(vcross(rB_cross_n, rB), n) * inv_inertia_b;
			const real m_eff = 1.0 / (termA + termB);

			// apply some baumgarte stabilizationw which adds some velocity bias to push bodies
			// apart if they are sinking into each other
			real bias = 0.0;
			if (c.penetration_depth > ALLOWED_PENETRATION)
				bias = (BAUMGARTE_BETA / dt) * (c.penetration_depth - ALLOWED_PENETRATION);

			const real lambda = (bias - v_rel_n) * m_eff;

			// clamp accumulated impulse
			// we cannot pull objects together so the total accumulated impulse must be >= 0
			const real old_impulse = c.accumulated_impulse;
			c.accumulated_impulse = fmax(old_impulse + lambda, 0.0);
			const real delta_lambda = c.accumulated_impulse - old_impulse;

			// now we want to apply the impulse vector to the bodies
			const vreal4 impulse_vector = n * delta_lambda;

			vA += impulse_vector * inv_mass_a;
			wA += vcross(rA, impulse_vector) * inv_inertia_a;
			vB -= impulse_vector * inv_mass_b;
			wB -= vcross(rB, impulse_vector) * inv_inertia_b;
		}
	}
}
