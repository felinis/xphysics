//
//	xphysics rigid body dynamics engine
//	(C) 2026 Alan Moczulski
//

#include "xp_narrowphase.hxx"
#include "xp_math_operators.hxx"

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
		const vreal4 b = simplex.points[1];
		const vreal4 c = simplex.points[2];
		const vreal4 ab = b - a;
		const vreal4 ac = c - a;
		const vreal4 a0 = -a;

		// compute triangle normal
		const vreal4 abc = vcross(ab, ac);

		// vector perpendicular to ac, points out of the triangle
		const vreal4 cross_abc_ac = vcross(abc, ac);

		// vector perpendicular to ab, points out of the triangle
		const vreal4 cross_ab_abc = vcross(ab, abc);

		if (vdot(cross_abc_ac, a0) > 0.0)
		{
			if (vdot(ac, a0) > 0.0)
			{
				// origin is perpendicular to ac, so we discard b
				simplex.points[1] = c; // simplex is now [a, c]
				simplex.count = 2;
				dir = vcross(vcross(ac, a0), ac);
			}
			else
			{
				// fallback to testing the ab region
				if (vdot(ab, a0) > 0.0)
				{
					simplex.points[1] = b; // simplex is now [a, b]
					simplex.count = 2;
					dir = vcross(vcross(ab, a0), ab);
				}
				else
				{
					// origin is closest to vertex a, so we discard b and c
					simplex.count = 1;
					dir = a0;
				}
			}
		}
		else if (vdot(cross_ab_abc, a0) > 0.0)
		{
			// origin is outside edge ab
			if (vdot(ab, a0) > 0.0)
			{
				simplex.points[1] = b; // simplex is now [a, b]
				simplex.count = 2;
				dir = vcross(vcross(ab, a0), ab);
			}
			else
			{
				simplex.count = 1;
				dir = a0;
			}
		}
		else
		{
			// origin is inside the triangular prism, not outside any edge
			// we just need to check if it's above or below
			if (vdot(abc, a0) > 0.0)
			{
				// origin is above the triangle in the directino of the normal
				dir = abc;
			}
			else
			{
				// origin is below the triangle in the directino of the normal
				dir = -abc;

				// important: we must swap b and c to maintain proper winding order
				// so the normal points correctly in the next iteration
				simplex.points[1] = c;
				simplex.points[2] = b;
			}
		}

		return false; // a triangle cannot enclose a 3d volume
	}
	else if (simplex.count == 4) // the simplex is a tetrahedron
	{
		const vreal4 b = simplex.points[1];
		const vreal4 c = simplex.points[2];
		const vreal4 d = simplex.points[3];

		const vreal4 ab = b - a;
		const vreal4 ac = c - a;
		const vreal4 ad = d - a;
		const vreal4 a0 = -a;

		// compute out facing normals for the 3 faces that share a
		// important: winding order to make sure that normals point out of tetrahedron
		const vreal4 abc = vcross(ab, ac);
		const vreal4 acd = vcross(ac, ad);
		const vreal4 adb = vcross(ad, ab);

		// check if origin is outside face abc
		if (vdot(abc, a0) > 0.0)
		{
			// origin is outside abc, we can discard d
			simplex.count = 3;
			dir = abc;
			return false;
		}
		if (vdot(acd, a0) > 0.0)
		{
			// origin is outside acd, we can discard b
			simplex.points[1] = c;
			simplex.points[2] = d;
			simplex.count = 3;
			dir = acd;
			return false;
		}
		if (vdot(adb, a0) > 0.0)
		{
			// origin is outside adb, we can discard c
			simplex.points[1] = d;
			simplex.points[2] = b;
			simplex.count = 3;
			dir = adb;
			return false;
		}

		// if we get here, the origin is not outside abc, acd or adb, and
		// since we already know it's not outside bcd, it has to be inside the tetrahedron
		return true; // collision detected
	}

	return false;
}

bool xp_gjk_intersect(const xp_convex_hull& hull_a, const xp_convex_hull& hull_b, xp_simplex& simplex)
{
	vreal4 search_direction = { 1.0, 0.0, 0.0, 0.0 }; // todo: cache and get the one from previous frame so we have O(1)

	// get the first point on the minkowski difference
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

// represents a single triangular face on the epa polytope
struct xp_epa_face
{
	vreal4 a, b, c;
	vreal4 normal;
	real distance;
};

// computes a face's normal and distance from the origin
inline void xp_compute_face_info(xp_epa_face& face)
{
	const vreal4 ab = face.b - face.a;
	const vreal4 ac = face.c - face.a;
	face.normal = vnormalize(vcross(ab, ac)); // important: winding order
	face.distance = vdot(face.normal, face.a);
}

struct xp_epa_edge
{
	vreal4 a, b;
};

inline bool xp_vequals(const vreal4& a, const vreal4& b)
{
	const vreal4 diff = a - b;
	return vdot(diff, diff) < 0.00001;
}

// add an edge to the silhouette list
inline void xp_add_epa_edge(xp_epa_edge* edges, u32& nedges, const vreal4& a, const vreal4& b)
{
	// check if edge already exists
	for (u32 i = 0; i < nedges; ++i)
	{
		// if the reverse edge exists, we have found a pair of faces that share this edge,
		// thus it is an internal edge, not part of the outer silhouette
		if (xp_vequals(edges[i].a, b) && xp_vequals(edges[i].b, a))
		{
			// remove the existing edge by swapping with the last element
			edges[i] = edges[nedges - 1];
			nedges--;
			return;
		}
	}
	
	// otherwise it's a new edge, add it to the silhouette
	edges[nedges++] = { a, b };
}

bool xp_epa_expand(
	const xp_convex_hull& hull_a,
	const xp_convex_hull& hull_b,
	xp_simplex& simplex,
	vreal4& normal,
	real& penetration_depth
)
{
	// initialize the polytope with the 4 faces of the GJK tetrahedron
	const vreal4 a = simplex.points[0];
	const vreal4 b = simplex.points[1];
	const vreal4 c = simplex.points[2];
	const vreal4 d = simplex.points[3];

	// allocate a flat array for out polytope faces from the transient arena
	const u32 MAX_FACES = 64;
	xp_epa_face faces[MAX_FACES];
	u32 nfaces = 0;

	// note: winding order must ensure that normals are pointing out
	faces[0] = { a, b, c };
	xp_compute_face_info(faces[0]);
	faces[1] = { a, c, d };
	xp_compute_face_info(faces[1]);
	faces[2] = { a, d, b };
	xp_compute_face_info(faces[2]);
	faces[3] = { b, c, d };
	xp_compute_face_info(faces[3]);
	nfaces = 4;

	// main epa iteration loop
	const u32 MAX_EPA_ITERATIONS = 32; // we need this to prevent infinite loops
	for (u32 i = 0; i < MAX_EPA_ITERATIONS; ++i)
	{
		// find the face closest to the origin
		u32 min_face_index = 0;
		real min_distance = faces[0].distance;
		for (u32 j = 1; j < nfaces; ++j)
		{
			if (faces[j].distance < min_distance)
			{
				min_distance = faces[j].distance;
				min_face_index = j;
			}
		}

		xp_epa_face closest_face = faces[min_face_index];

		// find a new point in the direction of the normal
		const vreal4 support_point = xp_get_minkowski_support(hull_a, hull_b, closest_face.normal);
		const real support_dist = vdot(closest_face.normal, support_point);

		// tolerance check: if the support point is not significantly further than the face,
		// we have found the bounds of the minkowski difference
		const real EPA_TOLERANCE = 0.01;
		if (support_dist - min_distance < EPA_TOLERANCE)
		{
			normal = closest_face.normal;
			penetration_depth = support_dist;
			return true; // collision detected
		}

		// expand the polytope
		xp_epa_edge silhouette[MAX_FACES * 3];
		u32 nedges = 0;

		// find all faces visible from the new support point and extract their edges
		for (u32 j = 0; j < nfaces; )
		{
			// a face is visible if it points towards the new support point
			if (vdot(faces[j].normal, support_point) > faces[j].distance + 0.0001)
			{
				// add the edges to the silhouette
				// important: winding order matters
				xp_add_epa_edge(silhouette, nedges, faces[j].a, faces[j].b);
				xp_add_epa_edge(silhouette, nedges, faces[j].b, faces[j].c);
				xp_add_epa_edge(silhouette, nedges, faces[j].c, faces[j].a);

				// delete the visible face by swapping it with the last face
				faces[j] = faces[nfaces - 1];
				nfaces--;
			}
			else
				j++; // face is not visible, move to the next one
		}

		// if we exceed out max face budget, break out and use the best normal we have
		if (nfaces + nedges >= MAX_FACES)
			break;

		// stitch new faces from the silhouette edges to the new support point
		for (u32 j = 0; j < nedges; ++j)
		{
			faces[nfaces].a = silhouette[j].a;
			faces[nfaces].b = silhouette[j].b;
			faces[nfaces].c = support_point; // the new peak of the tent
			xp_compute_face_info(faces[nfaces]);
			nfaces++;
		}
	}

	// as a fallback, use the last best face
	normal = faces[0].normal;
	penetration_depth = faces[0].distance;

	return true; // collision detected
}
