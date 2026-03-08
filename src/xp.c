//
//	xphysics rigid body dynamics engine
//	(C) 2026 Alan Moczulski
//

#include "xp.h"

typedef struct
{
	u32 test;
} xp_context_internal;

xp_context xp_init(void* mem, usize size)
{
	return (xp_context*)mem;
}

void xp_shutdown(xp_context xpc)
{
	
}

void xp_update(xp_context xpc, real dt)
{
	
}
