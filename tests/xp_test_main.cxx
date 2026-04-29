#include "xp.h"
#include <stdio.h>
#include <stdlib.h>

int main()
{
	const usize persistent_size = XPGetPersistentMemoryRequirements(1024, 128);
	const usize transient_size = XPGetTransientMemoryRequirements(128, 128);
	u8* mem = (u8*)malloc(persistent_size + transient_size);

	XPMemoryProvider xmp;
	xmp.persistent_memory = mem;
	xmp.persistent_size = persistent_size;
	xmp.transient_memory = mem + persistent_size;
	xmp.transient_size = transient_size;

	int result = 0;

	if (XPContext* xpc = XPInit(&xmp, 64, 128))
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

		id cube_shape = XPCreateConvexHull(xpc, cube_vertices, 8);
		id floor_shape = XPCreateConvexHull(xpc, floor_vertices, 8);

		id floor_body = XPCreateFixedBody(xpc);
		XPAttachShape(xpc, floor_body, floor_shape);
		const real floor_pos[3] = {0.0, 0.0, 0.0};
		XPSetBodyPosition(xpc, floor_body, floor_pos);

		id cube_body = XPCreateDynamicBody(xpc, 10.0);
		XPAttachShape(xpc, cube_body, cube_shape);
		const real cube_pos[3] = {0.0, 0.0, 3.0};
		XPSetBodyPosition(xpc, cube_body, cube_pos);
		printf("Cube height:\n3.0\n");

//		const real gravity[3] = { 0.0, 0.0, -9.81 };
//		XPSetGravity(xpc, gravity);

		for (int i = 0; i < 120; ++i)
		{
			XPStep(xpc, 1.0 / 60.0);

			real body_position[3];
			XPGetBodyPosition(xpc, cube_body, body_position);
			printf("%.3f\n", body_position[2]);
		}
#if 0
		if (final_pos[2] < -0.1) // we allow slight penetration due to PGS solver tolerance, but it should not be very negative
		{
			printf("TEST FAILED: cube fell through the floor.\n");
			result = 1;
		}
		else
		{
			printf("TEST PASSED: cube is resting on the floor.\n");
		}
#endif
		XPDestroyBody(xpc, cube_body);
		XPDestroyBody(xpc, floor_body);
	}
	else
	{
		printf("TEST FAILED: Failed to initialize xphysics.\n");
		result = 1;
	}

	free(mem);

	return result;
}
