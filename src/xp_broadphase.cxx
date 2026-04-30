#include "xp_broadphase.hxx"
#include <algorithm>

// world bounds
constexpr real WORLD_MIN = -1000.0;
constexpr real WORLD_MAX = 1000.0;
constexpr real WORLD_EXTENT = WORLD_MAX - WORLD_MIN;

// helper function to quantize a floating-point value
static inline u16 QuantizeCoord(real value)
{
	// clamp value to world bounds
	if (value < WORLD_MIN) value = WORLD_MIN;
	if (value > WORLD_MAX) value = WORLD_MAX;

	// quantize coordinate to 16-bit
	const real percentage = (value - WORLD_MIN) / WORLD_EXTENT;
	return static_cast<u16>(percentage * 65535.0);
}

// comparator for qsort
static inline bool AaBbCompareX(const XPBox& a, const XPBox& b)
{
	return a.min[0] < b.min[0];
}

MemoryRange<XPBroadPair> XPBroadSweepAndPrune(u32 active_body_count, const vreal4* body_positions, const qreal* body_orientations, const id* body_shape_ids, const XPConvexHull* hulls, LinearAllocator<u8>& transient_arena)
{
	// we need to allocate space for the aabbs in the transient arena
	const usize aabb_arena_size = sizeof(XPBox) * active_body_count;
	XPBox* aabbs = (XPBox*)transient_arena.PushBytes(aabb_arena_size);

	if (!aabbs)
	{
		XP_BREAK("Failed to allocate space for aabbs in the transient arena.");
		return {};
	}

	// compute and quantize aabbs from the actual convex hull vertices
	for (u32 i = 0; i < active_body_count; ++i)
	{
		const vreal4& pos = body_positions[i];
		const id hull_id = body_shape_ids[i];

		if (hull_id != INVALID_ID)
		{
			const XPConvexHull& hull = hulls[hull_id];
			const qreal& orient = body_orientations[i];

			// rotate each hull vertex and find min/max extents
			vreal4 vert0 = { hull.verts[0], hull.verts[1], hull.verts[2], 0.0 };
			vreal4 rotated0 = qrotate(orient, vert0);
			real aabb_min[3] = { rotated0.x, rotated0.y, rotated0.z };
			real aabb_max[3] = { rotated0.x, rotated0.y, rotated0.z };

			for (usize v = 1; v < hull.nverts; ++v)
			{
				vreal4 vert = { hull.verts[v * 3], hull.verts[v * 3 + 1], hull.verts[v * 3 + 2], 0.0 };
				vreal4 rotated = qrotate(orient, vert);

				if (rotated.x < aabb_min[0]) aabb_min[0] = rotated.x;
				if (rotated.y < aabb_min[1]) aabb_min[1] = rotated.y;
				if (rotated.z < aabb_min[2]) aabb_min[2] = rotated.z;
				if (rotated.x > aabb_max[0]) aabb_max[0] = rotated.x;
				if (rotated.y > aabb_max[1]) aabb_max[1] = rotated.y;
				if (rotated.z > aabb_max[2]) aabb_max[2] = rotated.z;
			}

			aabbs[i].min[0] = QuantizeCoord(pos.data[0] + aabb_min[0]);
			aabbs[i].min[1] = QuantizeCoord(pos.data[1] + aabb_min[1]);
			aabbs[i].min[2] = QuantizeCoord(pos.data[2] + aabb_min[2]);
			aabbs[i].max[0] = QuantizeCoord(pos.data[0] + aabb_max[0]);
			aabbs[i].max[1] = QuantizeCoord(pos.data[1] + aabb_max[1]);
			aabbs[i].max[2] = QuantizeCoord(pos.data[2] + aabb_max[2]);
		}
		else
		{
			// no hull attached, use a unit-sized fallback
			aabbs[i].min[0] = QuantizeCoord(pos.data[0] - 0.5);
			aabbs[i].min[1] = QuantizeCoord(pos.data[1] - 0.5);
			aabbs[i].min[2] = QuantizeCoord(pos.data[2] - 0.5);
			aabbs[i].max[0] = QuantizeCoord(pos.data[0] + 0.5);
			aabbs[i].max[1] = QuantizeCoord(pos.data[1] + 0.5);
			aabbs[i].max[2] = QuantizeCoord(pos.data[2] + 0.5);
		}

		aabbs[i].body_id = i;
	}

	// sort aabbs along the x axis
	// todo: investigate better performing sorting algorithms (like insertion sort if list is almost sorted)
	std::sort(aabbs, aabbs + active_body_count, AaBbCompareX);

	// now we need to go through the sorted aabbs and find overlaps
	// we will put them as we find them in the transient arena (so we don't know the initial size)
	XPBroadPair* pairs = (XPBroadPair*)(transient_arena.memory + transient_arena.size);
	u32 npairs = 0;

	for (u32 i = 0; i < active_body_count; ++i)
	{
		const XPBox& a = aabbs[i];

		for (u32 j = i + 1; j < active_body_count; ++j)
		{
			const XPBox& b = aabbs[j];

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
				XPBroadPair* new_pair = (XPBroadPair*)transient_arena.PushBytes(sizeof(XPBroadPair));
				if (new_pair)
				{
					new_pair->body_id_a = a.body_id;
					new_pair->body_id_b = b.body_id;
					npairs++;
				}
			}
		}
	}

	XPBroadPair* pairs_end = pairs + npairs;
	return { pairs, pairs_end };
}
