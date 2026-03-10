#pragma once

struct xp_convex_hull;

bool xp_gjk_intersect(const xp_convex_hull& hull_a, const xp_convex_hull& hull_b);
