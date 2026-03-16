//
//	xphysics rigid body dynamics engine
//	(C) 2026 Alan Moczulski
//

#include "xp.hxx"
#include "xp_broadphase.hxx"
#include "xp_internal.hxx"
#include "xp_math.hxx"
#include "xp_math_operators.hxx"
#include "xp_narrowphase.hxx"
#include "xp_solver.hxx"

static linear_allocator<u8> persistent_arena;
static linear_allocator<u8> transient_arena;

// rigid body SoA components
static u32 active_body_count = 0;
static linear_allocator<vreal4> positions;
static linear_allocator<vreal4> linear_velocities;
static linear_allocator<qreal> orientations;
static linear_allocator<qreal> angular_velocities;
static linear_allocator<real> inv_masses;
static linear_allocator<real> inv_inertias;

// convex hull storage
static linear_allocator<real> convex_hulls_verts;
static linear_allocator<xp_convex_hull> convex_hulls;
static u32 convex_hull_count = 0;

// maps body id -> convex hull id
static linear_allocator<id> body_hull_ids;

usize xp_get_persistent_memory_requirements(u32 num_convex_hull_verts, u32 num_bodies)
{
	return sizeof(real) * num_convex_hull_verts * 3 +
		sizeof(xp_convex_hull) * num_bodies +
		sizeof(vreal4) * num_bodies * 2 +	// positions + linear_velocities
		sizeof(qreal) * num_bodies * 2 +	// orientations + angular_velocities
		sizeof(real) * num_bodies * 2 +		// inv_masses + inv_inertias
		sizeof(id) * num_bodies;			// body_hull_ids
}

usize xp_get_transient_memory_requirements(u32 num_contacts, u32 num_bodies)
{
	return sizeof(xp_contact_manifold) * num_contacts +
		sizeof(xp_aabb) * num_bodies +
		sizeof(xp_broadphase_pair) * num_contacts;
}

bool xp_init(const memory_provider* mp, u32 convex_hulls_verts_budget, u32 bodies_budget)
{
	// validate memory
	if (!mp->persistent_memory || !mp->transient_memory)
		return false;

	// initialize memory arenas
	persistent_arena = linear_allocator<u8>(mp->persistent_memory, mp->persistent_size);
	transient_arena = linear_allocator<u8>(mp->transient_memory, mp->transient_size);

	// allocate convex hull vertex storage
	const usize verts_size = sizeof(real) * convex_hulls_verts_budget * 3;
	convex_hulls_verts = linear_allocator<real>((real*)persistent_arena.push_bytes(verts_size), verts_size);

	// allocate convex hull descriptors
	const usize hulls_size = sizeof(xp_convex_hull) * bodies_budget;
	convex_hulls = linear_allocator<xp_convex_hull>((xp_convex_hull*)persistent_arena.push_bytes(hulls_size), hulls_size);

	// allocate body SoA arrays
	const usize pos_size = sizeof(vreal4) * bodies_budget;
	positions = linear_allocator<vreal4>((vreal4*)persistent_arena.push_bytes(pos_size), pos_size);

	const usize vel_size = sizeof(vreal4) * bodies_budget;
	linear_velocities = linear_allocator<vreal4>((vreal4*)persistent_arena.push_bytes(vel_size), vel_size);

	const usize orient_size = sizeof(qreal) * bodies_budget;
	orientations = linear_allocator<qreal>((qreal*)persistent_arena.push_bytes(orient_size), orient_size);

	const usize angvel_size = sizeof(qreal) * bodies_budget;
	angular_velocities = linear_allocator<qreal>((qreal*)persistent_arena.push_bytes(angvel_size), angvel_size);

	const usize mass_size = sizeof(real) * bodies_budget;
	inv_masses = linear_allocator<real>((real*)persistent_arena.push_bytes(mass_size), mass_size);

	const usize inertia_size = sizeof(real) * bodies_budget;
	inv_inertias = linear_allocator<real>((real*)persistent_arena.push_bytes(inertia_size), inertia_size);

	// allocate body-to-hull mapping
	const usize hull_ids_size = sizeof(id) * bodies_budget;
	body_hull_ids = linear_allocator<id>((id*)persistent_arena.push_bytes(hull_ids_size), hull_ids_size);

	active_body_count = 0;
	convex_hull_count = 0;

	return true;
}

void xp_uninit()
{
	// nothing to do since memory is owned by the caller
}

static void xp_integrate(second dt)
{
	const vreal4 gravity = { 0.0, 0.0, -9.81, 0.0 };

	for (u32 i = 0; i < active_body_count; ++i)
	{
		if (inv_masses[i] == 0.0)
			continue; // skip static bodies

		vreal4 lin_vel = linear_velocities[i];
		qreal ang_vel = angular_velocities[i];

		vreal4 gravity_step = gravity * dt;
		lin_vel += gravity_step;
		vreal4 pos_step = lin_vel * dt;
		positions[i] += pos_step;

		linear_velocities[i] = lin_vel;

		const qreal q_ang_vel = angular_velocities[i];
		const qreal q_spin = qmul(q_ang_vel, orientations[i]) * 0.5 * dt;
		orientations[i] += q_spin;
		orientations[i] = qnormalize(orientations[i]);
	}
}

void xp_step(second dt)
{
	// reset transient memory for the frame
	transient_arena.reset();

	// perform broadphase aabb collision detection
	// todo: can we optimize by only passing in bodies that are not sleeping?
	const memory_range<xp_broadphase_pair> broadphase_pairs = xp_broadphase_sweep_and_prune(active_body_count, positions.memory, transient_arena);
	const u32 npairs = (u32)broadphase_pairs.size();

	// perform narrowphase gjk and epa collision detection
	// allocate space for contact manifolds in transient memory
	xp_contact_manifold* manifolds = (xp_contact_manifold*)transient_arena.push_bytes(sizeof(xp_contact_manifold) * npairs);
	u32 nmanifolds = 0;

	for (u32 i = 0; i < npairs; ++i)
	{
		const id id_a = broadphase_pairs[i].body_id_a;
		const id id_b = broadphase_pairs[i].body_id_b;

		const id hull_id_a = body_hull_ids[id_a];
		const id hull_id_b = body_hull_ids[id_b];

		// skip pairs where either body has no hull attached
		if (hull_id_a == INVALID_ID || hull_id_b == INVALID_ID)
			continue;

		const xp_convex_hull& hull_a = convex_hulls[hull_id_a];
		const xp_convex_hull& hull_b = convex_hulls[hull_id_b];

		xp_simplex simplex;
		if (xp_gjk_intersect(hull_a, hull_b, simplex))
		{
			vreal4 normal;
			real penetration_depth;
			if (xp_epa_expand(hull_a, hull_b, simplex, normal, penetration_depth))
			{
				xp_contact_manifold& m = manifolds[nmanifolds++];
				m.body_a = id_a;
				m.body_b = id_b;
				m.normal = normal;
				m.penetration_depth = penetration_depth;
				// hack: approximate contact point as midpoint between the two body centers shifted along the normal
				m.pos = positions[id_a] + normal * (penetration_depth * 0.5);
				m.accumulated_impulse = 0.0;
			}
		}
	}

	// collision resolution using pgs solver
	if (nmanifolds > 0)
	{
		xp_solve_contacts(
			nmanifolds,
			manifolds,
			positions.memory,
			linear_velocities.memory,
			(vreal4*)angular_velocities.memory, // cast qreal* to vreal4* as they have the same layout
			inv_masses.memory,
			inv_inertias.memory,
			dt
		);
	}

	// integrate positions and orientations
	xp_integrate(dt);
}

id xp_create_fixed_body()
{
	const id body_id = active_body_count++;

	const vreal4 zero_v = { 0.0, 0.0, 0.0, 0.0 };
	const qreal identity_q = { 0.0, 0.0, 0.0, 1.0 };

	positions[body_id] = zero_v;
	linear_velocities[body_id] = zero_v;
	orientations[body_id] = identity_q;
	angular_velocities[body_id] = { 0.0, 0.0, 0.0, 0.0 };
	inv_masses[body_id] = 0.0; // infinite mass (static)
	inv_inertias[body_id] = 0.0; // infinite inertia (static)
	body_hull_ids[body_id] = INVALID_ID;

	return body_id;
}

id xp_create_dynamic_body(kilogram mass)
{
	if (mass <= 0.0)
		return INVALID_ID;

	const id body_id = active_body_count++;

	const vreal4 zero_v = { 0.0, 0.0, 0.0, 0.0 };
	const qreal identity_q = { 0.0, 0.0, 0.0, 1.0 };

	positions[body_id] = zero_v;
	linear_velocities[body_id] = zero_v;
	orientations[body_id] = identity_q;
	angular_velocities[body_id] = { 0.0, 0.0, 0.0, 0.0 };
	inv_masses[body_id] = 1.0 / mass;
	// hack: we use a simplified scalar inertia for now, uniform sphere
	inv_inertias[body_id] = 5.0 / (2.0 * mass);
	body_hull_ids[body_id] = INVALID_ID;

	return body_id;
}

void xp_destroy_body(id body_id)
{
	// todo: implement proper slot reuse with a free list
	(void)body_id;
}

id xp_create_convex_hull(const real* vertex_positions, u32 vertex_count)
{
	// copy vertex data into the persistent convex hull vertex pool
	const usize verts_bytes = sizeof(real) * vertex_count * 3;
	real* dest = (real*)convex_hulls_verts.push_bytes(verts_bytes);
	if (!dest)
		return INVALID_ID;

	// copy vertex data
	for (u32 i = 0; i < vertex_count * 3; ++i)
		dest[i] = vertex_positions[i];

	// create a hull descriptor
	const id hull_id = convex_hull_count++;
	convex_hulls[hull_id].nverts = vertex_count;
	convex_hulls[hull_id].verts = dest;

	return hull_id;
}

void xp_attach_shape(id body_id, id shape_id)
{
	if (body_id < active_body_count)
		body_hull_ids[body_id] = shape_id;
}

void xp_get_body_position(id body_id, real out_position[3])
{
	out_position[0] = positions[body_id].data[0];
	out_position[1] = positions[body_id].data[1];
	out_position[2] = positions[body_id].data[2];
}

void xp_set_body_position(id body_id, const real position[3])
{
	positions[body_id].data[0] = position[0];
	positions[body_id].data[1] = position[1];
	positions[body_id].data[2] = position[2];
}
