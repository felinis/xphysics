//
//	xphysics rigid body dynamics engine
//	(C) 2026 Alan Moczulski
//

#include "xp.h"
#include "xp_broadphase.hxx"
#include "xp_internal.hxx"
#include "xp_math.hxx"
#include "xp_math_operators.hxx"
#include "xp_narrowphase.hxx"
#include "xp_solver.hxx"

struct XPContext
{
	// these hold the one and only memory allocation that will be used throughout the whole lifetime of the physics engine
	LinearAllocator<u8> persistent_arena;
	LinearAllocator<u8> transient_arena;

	// optional user-defined function to inform about errors
	void (*error_report_func)(const char* str);

	vreal4 gravity;

	// rigid body solver components
	u32 bodies_capacity;
	u32 bodies_count;
	vreal4* positions;
	vreal4* linear_velocities;
	qreal* orientations;
	qreal* angular_velocities;
	real* inv_masses;
	vreal4* inv_inertias; // packed diagonal components x, y, z of the inetria tensor

	// collision components
	u32 convex_hulls_capacity;
	u32 convex_hulls_count;
	real* convex_hulls_verts; // contains all convex hulls' verts
	XPConvexHull* convex_hulls;

	// todo: heightmap storage

	// maps body id -> convex hull id
	id* body_shape_ids;
};

usize XPGetPersistentMemoryRequirements(u32 num_convex_hull_verts, u32 num_bodies)
{
	return sizeof(real) * num_convex_hull_verts * 3 +
		sizeof(XPConvexHull) * num_bodies +
		sizeof(vreal4) * num_bodies * 2 +	// positions + linear_velocities
		sizeof(qreal) * num_bodies * 2 +	// orientations + angular_velocities
		sizeof(real) * num_bodies * 2 +		// inv_masses + inv_inertias
		sizeof(id) * num_bodies;			// body_shape_ids
}

usize XPGetTransientMemoryRequirements(u32 num_contacts, u32 num_bodies)
{
	return sizeof(XPContactManifold) * num_contacts +
		sizeof(XPBox) * num_bodies +
		sizeof(XPBroadPair) * num_contacts;
}

void XPReport(const XPContext* xpc, const char* str)
{
#if XP_CHECKED_BUILD
	if (xpc->error_report_func)
		xpc->error_report_func(str);
#endif
}

XPContext* XPInit(const XPMemoryProvider* mp, const XPReporter* reporter, u32 convex_hulls_verts_budget, u32 bodies_budget)
{
#if XP_CHECKED_BUILD
	// validate memory
	if (!mp->persistent_memory || !mp->transient_memory)
		return nullptr;

	// todo: validate memory sizes?
#endif

	// initialize memory arenas
	auto persistent_arena = LinearAllocator<u8>(mp->persistent_memory, mp->persistent_size);
	auto transient_arena = LinearAllocator<u8>(mp->transient_memory, mp->transient_size);

	// start data type definition at persistent memory head
	XPContext* xpc = (XPContext*)persistent_arena.PushBytes(sizeof(XPContext));
	xpc->persistent_arena = persistent_arena;
	xpc->transient_arena = transient_arena;

#if XP_CHECKED_BUILD
	if (reporter)
		xpc->error_report_func = reporter->error_report_func;
#endif

	// default gravity
	xpc->gravity = { 0.0, 0.0, -9.81, 0.0 };

	xpc->bodies_capacity = bodies_budget;
	xpc->bodies_count = 0;

	// allocate body SoA arrays
	const usize pos_size = sizeof(vreal4) * bodies_budget;
	xpc->positions = (vreal4*)persistent_arena.PushBytes(pos_size);

	const usize vel_size = sizeof(vreal4) * bodies_budget;
	xpc->linear_velocities = (vreal4*)persistent_arena.PushBytes(vel_size);

	const usize orient_size = sizeof(qreal) * bodies_budget;
	xpc->orientations = (qreal*)persistent_arena.PushBytes(orient_size);

	const usize angvel_size = sizeof(qreal) * bodies_budget;
	xpc->angular_velocities = (qreal*)persistent_arena.PushBytes(angvel_size);

	const usize mass_size = sizeof(real) * bodies_budget;
	xpc->inv_masses = (real*)persistent_arena.PushBytes(mass_size);

	const usize inertia_size = sizeof(vreal4) * bodies_budget;
	xpc->inv_inertias = (vreal4*)persistent_arena.PushBytes(inertia_size);

	xpc->convex_hulls_capacity = convex_hulls_verts_budget;
	xpc->convex_hulls_count = 0;

	// allocate convex hull vertex storage
	const usize verts_size = sizeof(real) * convex_hulls_verts_budget * 3;
	xpc->convex_hulls_verts = (real*)persistent_arena.PushBytes(verts_size);

	// allocate convex hull descriptors
	const usize hulls_size = sizeof(XPConvexHull) * bodies_budget;
	xpc->convex_hulls = (XPConvexHull*)persistent_arena.PushBytes(hulls_size);

	// allocate body-to-hull mapping
	const usize hull_ids_size = sizeof(id) * bodies_budget;
	xpc->body_shape_ids = (id*)persistent_arena.PushBytes(hull_ids_size);

	return xpc;
}

void XPUninit(XPContext* xpc)
{
	// nothing to do since memory is owned by the caller
}

void XPSetGravity(XPContext* xpc, const real gravity[3])
{
	xpc->gravity.data[0] = gravity[0];
	xpc->gravity.data[1] = gravity[1];
	xpc->gravity.data[2] = gravity[2];
}

static void XPIntegrate(XPContext* xpc, second dt)
{
	for (u32 i = 0; i < xpc->bodies_count; ++i)
	{
		if (xpc->inv_masses[i] == 0.0)
			continue; // skip static bodies

		vreal4 lin_vel = xpc->linear_velocities[i];

		const vreal4 gravity_step = xpc->gravity * dt;
		lin_vel += gravity_step;
		const vreal4 pos_step = lin_vel * dt;
		xpc->positions[i] += pos_step;

		xpc->linear_velocities[i] = lin_vel;

		const qreal q_ang_vel = xpc->angular_velocities[i];
		const qreal q_spin = qmul(q_ang_vel, xpc->orientations[i]) * 0.5 * dt;
		xpc->orientations[i] += q_spin;
		xpc->orientations[i] = qnormalize(xpc->orientations[i]);
	}
}

void XPStep(XPContext* xpc, second dt)
{
	// the transient arena would hold any temporary calculation that's useful to keep during a single frame,
	// so the data that ends up in it has "frame lifetime"
	xpc->transient_arena.Reset();

	// perform broadphase aabb collision detection
	// todo: can we optimize by only passing in bodies that are not sleeping?
	const MemoryRange<XPBroadPair> broadphase_pairs = XPBroadSweepAndPrune(xpc->bodies_count, xpc->positions, xpc->orientations, xpc->body_shape_ids, xpc->convex_hulls, xpc->transient_arena);
	const u32 npairs = (u32)broadphase_pairs.GetCount();

	// perform narrowphase gjk and epa collision detection
	// allocate space for contact manifolds in transient memory
	XPContactManifold* manifolds = (XPContactManifold*)xpc->transient_arena.PushBytes(sizeof(XPContactManifold) * npairs);
	u32 nmanifolds = 0;

	for (u32 i = 0; i < npairs; ++i)
	{
		const id id_a = broadphase_pairs[i].body_id_a;
		const id id_b = broadphase_pairs[i].body_id_b;

		const id hull_id_a = xpc->body_shape_ids[id_a];
		const id hull_id_b = xpc->body_shape_ids[id_b];

		// skip pairs where either body has no hull attached
		if (hull_id_a == INVALID_ID || hull_id_b == INVALID_ID)
			continue;

		const XPConvexHull& hull_a = xpc->convex_hulls[hull_id_a];
		const XPConvexHull& hull_b = xpc->convex_hulls[hull_id_b];

		xp_simplex simplex;
		if (xp_gjk_intersect(hull_a, xpc->positions[id_a], xpc->orientations[id_a], hull_b, xpc->positions[id_b], xpc->orientations[id_b], simplex))
		{
			vreal4 normal;
			real penetration_depth;
			if (xp_epa_expand(hull_a, xpc->positions[id_a], xpc->orientations[id_a], hull_b, xpc->positions[id_b], xpc->orientations[id_b], simplex, normal, penetration_depth))
			{
				XPContactManifold& m = manifolds[nmanifolds++];
				m.body_a = id_a;
				m.body_b = id_b;
				m.normal = normal; // we need to negate the normal since epa returns outward normal of minkowski(a - b) but the solver expects normal from b to a
				m.penetration_depth = penetration_depth;
				// approximate contact point on body B's surface along the collision normal
				m.pos = xpc->positions[id_b] - m.normal * (penetration_depth * 0.5);
				m.accumulated_impulse = 0.0;
			}
		}
	}

	// collision resolution using pgs solver
	if (nmanifolds > 0)
	{
		XPSolveContacts(
			nmanifolds,
			manifolds,
			xpc->positions,
			xpc->linear_velocities,
			(vreal4*)xpc->angular_velocities, // cast qreal* to vreal4* as they have the same layout
			xpc->inv_masses,
			xpc->inv_inertias,
			dt
		);
	}

	// integrate positions and orientations
	XPIntegrate(xpc, dt);
}

id XPCreateFixedBody(XPContext* xpc)
{
	// todo: check capacity
	const id body_id = xpc->bodies_count++;

	const vreal4 zero_v = { 0.0, 0.0, 0.0, 0.0 };
	const qreal identity_q = { 0.0, 0.0, 0.0, 1.0 };

	xpc->positions[body_id] = zero_v;
	xpc->linear_velocities[body_id] = zero_v;
	xpc->orientations[body_id] = identity_q;
	xpc->angular_velocities[body_id] = { 0.0, 0.0, 0.0, 0.0 };
	xpc->inv_masses[body_id] = 0.0; // infinite mass (static)
	xpc->inv_inertias[body_id] = { 0.0, 0.0, 0.0, 0.0 }; // infinite inertia (static)
	xpc->body_shape_ids[body_id] = INVALID_ID; // by default, no shape assigned

	return body_id;
}

id XPCreateDynamicBody(XPContext* xpc, kilogram mass)
{
	if (mass <= 0.0)
	{
		XPReport(xpc, "Cannot create a dynamic body with null mass.\n");
		return INVALID_ID;
	}

	// todo: check capacity
	const id body_id = xpc->bodies_count++;
	const vreal4 zero_v = { 0.0, 0.0, 0.0, 0.0 };
	const qreal identity_q = { 0.0, 0.0, 0.0, 1.0 };

	xpc->positions[body_id] = zero_v;
	xpc->linear_velocities[body_id] = zero_v;
	xpc->orientations[body_id] = identity_q;
	xpc->angular_velocities[body_id] = { 0.0, 0.0, 0.0, 0.0 };
	xpc->inv_masses[body_id] = 1.0 / mass;

//	if (shape_type == ShapeType::ConvexHull)
//	{
//		const XPConvexHull* ch = (XPConvexHull*)shape;
		// hack: we use a simplified scalar inertia for now, uniform sphere
		vreal4 inv_inertia = { 5.0 / (2.0 * mass), 5.0 / (2.0 * mass), 5.0 / (2.0 * mass), 0 };
		xpc->inv_inertias[body_id] = inv_inertia;
//	}
#if 0
	else if (shape_type == ShapeType::Box)
	{
		const OrientedBox* ob = (OrientedBox*)shape;

		// compute box inertia
		//const real volume = 8.0 * ob->half_extents[0] * ob->half_extents[1] * ob->half_extents[2];

		const real ex2 = ob->half_extents[0] * ob->half_extents[0];
		const real ey2 = ob->half_extents[1] * ob->half_extents[1];
		const real ez2 = ob->half_extents[2] * ob->half_extents[2];

		vreal4 inv_inertia = { 0, 0, 0, 0 };
		inv_inertia.x = 1.0 / ((1.0 / 3.0) * mass * (ey2 + ez2));
		inv_inertia.y = 1.0 / ((1.0 / 3.0) * mass * (ex2 + ez2));
		inv_inertia.z = 1.0 / ((1.0 / 3.0) * mass * (ex2 + ey2));

		xpc->inv_inertias[body_id] = inv_inertia;
	}
#endif

	xpc->body_shape_ids[body_id] = INVALID_ID;

	return body_id;
}

void XPDestroyBody(XPContext* xpc, id body_id)
{
	// todo: implement proper slot reuse with a free list
	(void)body_id;
}

id XPCreateConvexHull(XPContext* xpc, const real* vertex_positions, u32 vertex_count)
{
	const id shape_id = xpc->convex_hulls_count++;

	// create a hull descriptor
	xpc->convex_hulls[shape_id].nverts = vertex_count;
	xpc->convex_hulls[shape_id].verts = xpc->convex_hulls_verts;

	// copy vertex data into the persistent convex hull vertex pool
	const usize vertex_positions_count = vertex_count * 3;
	for (usize i = 0; i < vertex_positions_count; ++i)
		*xpc->convex_hulls_verts++ = *vertex_positions++;

	return shape_id;
}

void XPAttachShape(XPContext* xpc, id body_id, id shape_id)
{
	if (body_id < xpc->bodies_capacity)
		xpc->body_shape_ids[body_id] = shape_id;
}

void XPGetBodyPosition(XPContext* xpc, id body_id, real out_position[3])
{
	out_position[0] = xpc->positions[body_id].data[0];
	out_position[1] = xpc->positions[body_id].data[1];
	out_position[2] = xpc->positions[body_id].data[2];
}

void XPSetBodyPosition(XPContext* xpc, id body_id, const real position[3])
{
	xpc->positions[body_id].data[0] = position[0];
	xpc->positions[body_id].data[1] = position[1];
	xpc->positions[body_id].data[2] = position[2];
}

void XPGetBodyLinearVelocity(XPContext* xpc, id body_id, real out_velocity[3])
{
	out_velocity[0] = xpc->linear_velocities[body_id].data[0];
	out_velocity[1] = xpc->linear_velocities[body_id].data[1];
	out_velocity[2] = xpc->linear_velocities[body_id].data[2];
}

void XPGetBodyOrientation(XPContext* xpc, id body_id, real out_orientation[4])
{
	out_orientation[0] = xpc->orientations[body_id].data[0];
	out_orientation[1] = xpc->orientations[body_id].data[1];
	out_orientation[2] = xpc->orientations[body_id].data[2];
	out_orientation[3] = xpc->orientations[body_id].data[3];
}
