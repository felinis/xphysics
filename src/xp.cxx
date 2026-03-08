//
//	xphysics rigid body dynamics engine
//	(C) 2026 Alan Moczulski
//

#include "xp.hxx"

template <typename T>
struct linear_allocator
{
	T* memory;
	usize size;
	usize offset;

	linear_allocator() : memory(nullptr), size(0), offset(0) {}
	linear_allocator(T* memory, usize size) : memory(memory), size(size), offset(0) {}

	void* push_bytes(usize count)
	{
		if (offset + count > size)
			return nullptr;

		void* result = reinterpret_cast<u8*>(memory) + offset;
		offset += count;
		return result;
	}

	void reset() { offset = 0; }

	T& operator[](usize i) { return memory[i]; }
	const T& operator[](usize i) const { return memory[i]; }
};
static linear_allocator<u8> persistent_arena;
static linear_allocator<u8> transient_arena;

template <typename T>
struct memory_range
{
	T* begin;
	T* end;
};

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

struct xp_body
{
	real position[3];
};
static linear_allocator<xp_body> bodies;

usize xp_get_memory_requirements(u32 num_convex_hull_verts, u32 num_contacts, u32 num_bodies)
{
	return sizeof(real) * num_convex_hull_verts +
		sizeof(xp_contact_manifold) * num_contacts +
		sizeof(xp_body) * num_bodies;
}

bool xp_init(const memory_provider* mp, u32 convex_hulls_verts_budget, u32 bodies_budget)
{
	// validate memory
	if (!mp->persistent_memory || !mp->transient_memory)
	{
		XP_BREAK("Invalid memory pointers provided.");
		return false;
	}
	persistent_arena = linear_allocator<u8>(mp->persistent_memory, mp->persistent_size);
	transient_arena = linear_allocator<u8>(mp->transient_memory, mp->transient_size);

	// distribute the arena memory across various systems
	const usize convex_hulls_verts_arena_size = sizeof(real) * convex_hulls_verts_budget;
	convex_hulls_verts = linear_allocator<real>((real*)persistent_arena.push_bytes(convex_hulls_verts_arena_size), convex_hulls_verts_arena_size);
	const usize bodies_arena_size = sizeof(real) * convex_hulls_verts_budget;
	bodies = linear_allocator<xp_body>((xp_body*)persistent_arena.push_bytes(bodies_arena_size), bodies_arena_size);

	// todo: contact manifolds

	return true;
}

void xp_uninit()
{

}

void xp_step(second dt)
{
	// reset transient memory for the frame
	transient_arena.reset();

	// todo: collision detection using contact manifolds, AABB sweep and prune, quicksort

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
	out_position[0] = bodies[body_id].position[0];
	out_position[1] = bodies[body_id].position[1];
	out_position[2] = bodies[body_id].position[2];
}

void xp_set_body_position(id body_id, const real position[3])
{
	bodies[body_id].position[0] = position[0];
	bodies[body_id].position[1] = position[1];
	bodies[body_id].position[2] = position[2];
}
