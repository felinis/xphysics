//
//	xphysics rigid body dynamics engine
//	(C) 2026 Alan Moczulski
//

#include "xp_narrowphase.hxx"
#include "xp_math.hxx"
#include "xp_math_operators.hxx"

struct xp_convex_hull
{
	usize nverts;
	real* verts; // all vertex positions
};

// support mapping function: finds the furthest vertex of the convex hull in a given direction 
inline vreal4 xp_get_convex_hull_support(const xp_convex_hull& hull, const vreal4& dir)
{
	vreal4 best_vert = { hull.verts[0], hull.verts[1], hull.verts[2], 0.0 };
	real max_dot = vdot(best_vert, dir);

	// iterate through remaining verts to find the furthest one
	// todo: possible simd optimizations with _mm256_cmp_pd and _mm256_blendv_pd
	for (usize i = 1; i < hull.nverts; i++)
	{
		const vreal4 vert = { hull.verts[i * 3], hull.verts[i * 3 + 1], hull.verts[i * 3 + 2], 0.0 };
		const real dot = vdot(vert, dir);
		if (dot > max_dot)
		{
			max_dot = dot;
			best_vert = vert;
		}
	}
	return best_vert;
}

// computes a point on the boundary of the minkowski difference of two convex hulls
inline vreal4 xp_get_minkowski_support(const xp_convex_hull& hull_a, const xp_convex_hull& hull_b, const vreal4& dir)
{
	vreal4 furthest_vert_of_a = xp_get_convex_hull_support(hull_a, dir);
	vreal4 furthest_vert_of_b = xp_get_convex_hull_support(hull_b, -dir); // opposite direction
	// todo: do we have to convert to world space here?
	return furthest_vert_of_a - furthest_vert_of_b;
}

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

bool xp_do_simplex(xp_simplex& simplex, vreal4& dir)
{
	const vreal4 a = simplex.points[0];

	if (simplex.count == 2) // the simplex is a line
	{
		const vreal4 b = simplex.points[1];
		const vreal4 ab = b - a; // line segment
		vreal4 a0 = -a; // vector from a towards the origin

		// check if the origin lies in the voronoi region of the line segment
		if (vdot(ab, a0) > 0.0)
		{
			// origin is perpendicular to the line segment
			// the new direction is the double cross product
			dir = vcross(vcross(ab, a0), ab);
		}
		else
		{
			// origin is in the voronoi region of vertex a
			// we can discard b since the simplex if now just a
			simplex.count = 1;
			dir = a0;
		}
		return false; // a line cannot enclose a 3d volume
	}
	else if (simplex.count == 3) // the simplex is a triangle
	{
		// todo
	}
	else if (simplex.count == 4) // the simplex is a tetrahedron
	{
		// todo
	}

	return false;
}

bool xp_gjk_intersect(const xp_convex_hull& hull_a, const xp_convex_hull& hull_b)
{
	vreal4 search_direction = { 1.0, 0.0, 0.0, 0.0 }; // todo: cache and get the one from previous frame so we have O(1)

	// get the first point on the minkowski difference
	xp_simplex simplex;
	simplex.count = 0;
	simplex.push(xp_get_minkowski_support(hull_a, hull_b, search_direction));

	// search towards the origin
	search_direction = -search_direction;

	// main gjk loop
	const u32 MAX_GJK_ITERATIONS = 64; // we need this to prevent infinite loops
	for (u32 i = 0; i < MAX_GJK_ITERATIONS; ++i)
	{
		const vreal4 point = xp_get_minkowski_support(hull_a, hull_b, search_direction);
		// if the furthesr point in the search direction does not pass the origin,
		// it means that the origin cannot be inside the minkowski difference
		if (vdot(point, search_direction) < 0.0)
			return false; // separating axis found, no collision

		simplex.push(point);

		if (xp_do_simplex(simplex, search_direction))
			return true; // simplex encloses the origin, we have interpenetration
	}

	return false;
}
