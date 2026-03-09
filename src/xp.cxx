//
//	xphysics rigid body dynamics engine
//	(C) 2026 Alan Moczulski
//

#include "xp.hxx"
#include "xp_broadphase.hxx"
#include "xp_internal.hxx"

static linear_allocator<u8> persistent_arena;
static linear_allocator<u8> transient_arena;

struct xp_contact_manifold
{
	real pos[3];
	real penetration_depth;
};
static linear_allocator<xp_contact_manifold> contact_manifolds;

struct xp_convex_hull
{
	memory_range<real> verts; // in convex_hulls_verts

};
static linear_allocator<xp_convex_hull> convex_hulls;

struct xp_hull_vert
{
	real pos[3];
};
static linear_allocator<real> convex_hulls_verts;

static linear_allocator<position> bodies_positions; // stores all rigid bodies' positions in world space

usize xp_get_memory_requirements(u32 num_convex_hull_verts, u32 num_contacts, u32 num_bodies)
{
	return sizeof(real) * num_convex_hull_verts +
		sizeof(xp_contact_manifold) * num_contacts;// +
//		sizeof(xp_body) * num_bodies;
}

bool xp_init(const memory_provider* mp, u32 convex_hulls_verts_budget, u32 bodies_budget)
{
	// validate memory
	if (!mp->persistent_memory || !mp->transient_memory)
		return false;

	// initialize memory arenas
	persistent_arena = linear_allocator<u8>(mp->persistent_memory, mp->persistent_size);
	transient_arena = linear_allocator<u8>(mp->transient_memory, mp->transient_size);

	// distribute the arenas across various systems
	const usize convex_hulls_verts_arena_size = sizeof(real) * convex_hulls_verts_budget;
	convex_hulls_verts = linear_allocator<real>((real*)persistent_arena.push_bytes(convex_hulls_verts_arena_size), convex_hulls_verts_arena_size);
//	const usize bodies_arena_size = sizeof(real) * convex_hulls_verts_budget;
//	bodies = linear_allocator<xp_body>((xp_body*)persistent_arena.push_bytes(bodies_arena_size), bodies_arena_size);

	// todo: contact manifolds

	return true;
}

void xp_uninit()
{
	// nothing to do
}

void xp_step(second dt)
{
	// reset transient memory for the frame
	transient_arena.reset();

	// perform broadphase aabb collision detection
	// todo: can we optimize by only passing in bodies that are not sleeping?
	const memory_range<xp_broadphase_pair> broadphase_bodies_pairs = xp_broadphase_sweep_and_prune(bodies_positions.size, bodies_positions.memory, transient_arena);

	// perform narrowphase gjk collision detection
	// iterate through pairs and generate contact manifolds
	// todo

	// todo: collision detection using contact manifolds

	// integrate
	// todo: semi-implicit euler
}

id xp_create_fixed_body()
{
	return INVALID_ID;
}

id xp_create_dynamic_body(kilogram mass)
{
	return INVALID_ID;
}

void xp_destroy_body(id body_id)
{
	
}

id xp_create_convex_hull(const real* vertex_positions, u32 vertex_count)
{


	return INVALID_ID;
}

void xp_attach_shape(id body_id, id shape_id)
{
	
}

void xp_get_body_position(id body_id, real out_position[3])
{
	out_position[0] = bodies_positions[body_id].coords[0];
	out_position[1] = bodies_positions[body_id].coords[1];
	out_position[2] = bodies_positions[body_id].coords[2];
}

void xp_set_body_position(id body_id, const real position[3])
{
	bodies_positions[body_id].coords[0] = position[0];
	bodies_positions[body_id].coords[1] = position[1];
	bodies_positions[body_id].coords[2] = position[2];
}
