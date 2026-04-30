#pragma once
#include "xp_internal.hxx"
#include "xp_math.hxx"

// 16-bit quantized aabb
struct XPBox
{
	u16 min[3];
	u16 max[3];
	id body_id;
};

// represents a potential collision pair found by the broadphase
struct XPBroadPair
{
	id body_id_a;
	id body_id_b;
};

MemoryRange<XPBroadPair> XPBroadSweepAndPrune(u32 active_body_count, const vreal4* body_positions, const qreal* body_orientations, const id* body_shape_ids, const XPConvexHull* hulls, LinearAllocator<u8>& transient_arena);
