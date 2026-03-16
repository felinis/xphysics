#include "xp_broadphase.hxx"
#include <algorithm>

// world bounds
constexpr real WORLD_MIN = -1000.0;
constexpr real WORLD_MAX = 1000.0;
constexpr real WORLD_EXTENT = WORLD_MAX - WORLD_MIN;

// helper function to quantize a floating-point value
inline u16 quantize_coordinate(real value)
{
	// clamp value to world bounds
	if (value < WORLD_MIN) value = WORLD_MIN;
	if (value > WORLD_MAX) value = WORLD_MAX;

	// quantize coordinate to 16-bit
	const real percentage = (value - WORLD_MIN) / WORLD_EXTENT;
	return static_cast<u16>(percentage * 65535.0);
}

// comparator for qsort
inline bool aabb_compare_x(const xp_aabb& a, const xp_aabb& b)
{
	return a.min[0] < b.min[0];
}

memory_range<xp_broadphase_pair> xp_broadphase_sweep_and_prune(u32 active_body_count, const vreal4* body_positions, linear_allocator<u8>& transient_arena)
{
	// we need to allocate space for the aabbs in the transient arena
	const usize aabb_arena_size = sizeof(xp_aabb) * active_body_count;
	xp_aabb* aabbs = (xp_aabb*)transient_arena.push_bytes(aabb_arena_size);

	if (!aabbs)
	{
		XP_BREAK("Failed to allocate space for aabbs in the transient arena.");
		return {};
	}

	// compute and quantize aabbs for all active bodies
	// temporarily we calculate the aabbs in a dummy way
	for (u32 i = 0; i < active_body_count; ++i)
	{
		const vreal4& pos = body_positions[i];
		const real half_extents = 1.0; // just a dummy value for now
		
		aabbs[i].min[0] = quantize_coordinate(pos.data[0] - half_extents);
		aabbs[i].min[1] = quantize_coordinate(pos.data[1] - half_extents);
		aabbs[i].min[2] = quantize_coordinate(pos.data[2] - half_extents);
		aabbs[i].max[0] = quantize_coordinate(pos.data[0] + half_extents);
		aabbs[i].max[1] = quantize_coordinate(pos.data[1] + half_extents);
		aabbs[i].max[2] = quantize_coordinate(pos.data[2] + half_extents);
		aabbs[i].body_id = i;
	}

	// sort aabbs along the x axis
	// todo: investigate better performing sorting algorithms
	std::sort(aabbs, aabbs + active_body_count, aabb_compare_x);

	// now we need to go through the sorted aabbs and find overlaps
	// we will put them as we find them in the transient arena (so we don't know the initial size)
	xp_broadphase_pair* pairs = (xp_broadphase_pair*)(transient_arena.memory + transient_arena.offset);
	u32 npairs = 0;

	for (u32 i = 0; i < active_body_count; ++i)
	{
		const xp_aabb& a = aabbs[i];

		for (u32 j = i + 1; j < active_body_count; ++j)
		{
			const xp_aabb& b = aabbs[j];

			// if b's minimum x is strictly greater than a's maximum x, it means that
			// no further bodies can overlap with a along the x axis
			if (b.min[0] > a.max[0])
				break;

			// if we're here, it means that a and b overlap along x
			// now let's check y and z
			const bool overlap_y = (a.min[1] <= b.max[1]) && (a.max[1] >= b.min[1]);
			const bool overlap_z = (a.min[2] <= b.max[2]) && (a.max[2] >= b.min[2]);

			if (overlap_y && overlap_z)
			{
				// push a new pair to the transient arena
				xp_broadphase_pair* new_pair = (xp_broadphase_pair*)transient_arena.push_bytes(sizeof(xp_broadphase_pair));
				if (new_pair)
				{
					new_pair->body_id_a = a.body_id;
					new_pair->body_id_b = b.body_id;
					npairs++;
				}
			}
		}
	}

	xp_broadphase_pair* pairs_end = pairs + npairs;
	return { pairs, pairs_end };
}
