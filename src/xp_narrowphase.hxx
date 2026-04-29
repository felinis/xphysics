#pragma once
#include "xp_internal.hxx"
#include "xp_math.hxx"

// defines a gjk shape
struct xp_simplex
{
	vreal4 points[4]; // these can form a point, line, triangle or tetrahedron
	u8 count;

	void push(vreal4 point)
	{
		// shift points to make room
		points[3] = points[2];
		points[2] = points[1];
		points[1] = points[0];
		points[0] = point;
		count = (count < 4) ? count + 1 : 4;
	}
};

bool xp_gjk_intersect(
	const XPConvexHull& hull_a, const vreal4& pos_a, const qreal& orient_a,
	const XPConvexHull& hull_b, const vreal4& pos_b, const qreal& orient_b,
	xp_simplex& simplex);

bool xp_epa_expand(
	const XPConvexHull& hull_a, const vreal4& pos_a, const qreal& orient_a,
	const XPConvexHull& hull_b, const vreal4& pos_b, const qreal& orient_b,
	xp_simplex& simplex,
	vreal4& normal,
	real& penetration_depth
);
