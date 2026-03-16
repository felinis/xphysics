#include "xp.h"
#include <stdio.h>
#include <stdlib.h>

int main()
{
	const usize persistent_size = xp_get_persistent_memory_requirements(1024, 128);
	const usize transient_size = xp_get_transient_memory_requirements(128, 128);
	u8* mem = reinterpret_cast<u8*>(malloc(persistent_size + transient_size));

	memory_provider mp;
	mp.persistent_memory = mem;
	mp.persistent_size = persistent_size;
	mp.transient_memory = mem + persistent_size;
	mp.transient_size = transient_size;

	int result = 0;

	if (xp_init(&mp, 64, 128))
	{
		const real cube_vertices[] = {
			-0.5, -0.5, -0.5,
			 0.5, -0.5, -0.5,
			-0.5,  0.5, -0.5,
			 0.5,  0.5, -0.5,
			-0.5, -0.5,  0.5,
			 0.5, -0.5,  0.5,
			-0.5,  0.5,  0.5,
			 0.5,  0.5,  0.5
		};

		const real floor_vertices[] = {
			-5.0, -5.0, -0.5,
			 5.0, -5.0, -0.5,
			-5.0,  5.0, -0.5,
			 5.0,  5.0, -0.5,
			-5.0, -5.0,  0.5,
			 5.0, -5.0,  0.5,
			-5.0,  5.0,  0.5,
			 5.0,  5.0,  0.5
		};

		id cube_shape = xp_create_convex_hull(cube_vertices, 8);
		id floor_shape = xp_create_convex_hull(floor_vertices, 8);

		id floor_body = xp_create_fixed_body();
		xp_attach_shape(floor_body, floor_shape);
		const real floor_pos[3] = {0.0, 0.0, -1.0};
		xp_set_body_position(floor_body, floor_pos);

		id cube_body = xp_create_dynamic_body(10.0);
		xp_attach_shape(cube_body, cube_shape);
		const real cube_pos[3] = {0.0, 0.0, 5.0};
		xp_set_body_position(cube_body, cube_pos);

		for (int i = 0; i < 120; ++i)
		{
			xp_step(1.0 / 60.0);
		}

		real final_pos[3];
		xp_get_body_position(cube_body, final_pos);

		printf("Final cube position: (%.3f, %.3f, %.3f)\n", final_pos[0], final_pos[1], final_pos[2]);

		if (final_pos[2] < -0.1) // we allow slight penetration due to PGS solver tolerance, but it should not be very negative
		{
			printf("TEST FAILED: cube fell through the floor.\n");
			result = 1;
		}
		else
		{
			printf("TEST PASSED: cube is resting on the floor.\n");
		}

		xp_destroy_body(cube_body);
		xp_destroy_body(floor_body);
	}
	else
	{
		printf("TEST FAILED: Failed to initialize xphysics.\n");
		result = 1;
	}

	free(mem);

	return result;
}
