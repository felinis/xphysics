#pragma once
#include "xp_internal.hxx"
#include "xp_math.hxx"

// 16-bit quantized AABB
struct xp_aabb
{
	u16 min[3];
	u16 max[3];
	id body_id;
};

// represents a potential collision pair found by the broadphase
struct xp_broadphase_pair
{
	id body_id_a;
	id body_id_b;
};

memory_range<xp_broadphase_pair> xp_broadphase_sweep_and_prune(u32 active_body_count, const vreal4* body_positions, linear_allocator<u8>& transient_arena);
