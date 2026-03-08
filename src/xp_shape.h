#pragma once
#include "xp_math.h"

struct xp_shape_internal
{
	usize nverts;
	real* verts; // all vertex positions
};

xp_shape xp_create_convex_hull_shape(xp_context context, real* vertex_positions, usize vertex_count)
{
	// todo: push the array given the context's memory
}

void xp_destroy_shape(xp_context context, xp_shape shape)
{

}
