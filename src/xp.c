//
//	xphysics rigid body dynamics engine
//	(C) 2026 Alan Moczulski
//

#include "xp.h"

typedef struct
{
	usize max_convex_hulls;
	usize max_rigid_bodies;
	usize convex_hull_count;
	usize rigid_body_count;
	void* convex_hulls;
	void* rigid_bodies;
} xp_context_internal;

xp_context xp_init(const xp_context_create_info* xpcci)
{
	// make sure the memory blocks are big enough
	if (xpcci->persistent_memory_size < sizeof(xp_context_internal) + sizeof(void*) * xpcci->max_convex_hulls + sizeof(void*) * xpcci->max_rigid_bodies)
	{
		return 0;
	}

	xp_context_internal* xpci = (xp_context_internal*)xpcci->persistent_memory; // we start to initialize ourselves at the start of the persistent memory block
	xpci->max_convex_hulls = xpcci->max_convex_hulls;
	xpci->max_rigid_bodies = xpcci->max_rigid_bodies;
	xpci->convex_hull_count = 0;
	xpci->rigid_body_count = 0;
	xpci->convex_hulls = xpcci->persistent_memory + sizeof(xp_context_internal); // convex hulls start right after ourselves
	xpci->rigid_bodies = xpcci->persistent_memory + sizeof(xp_context_internal) + sizeof(void*) * xpcci->max_convex_hulls; // rigid bodies start right after convex hulls
	return xpci;
}

void xp_shutdown(xp_context xpc)
{
	
}

void xp_update(xp_context xpc, real dt)
{
	
}
