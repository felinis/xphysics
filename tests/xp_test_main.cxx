#include "xp.hxx"
#include <stdlib.h>

int main()
{
	const usize required_mem_size = xp_get_memory_requirements(1024, 1024, 1024);

	u8* mem = reinterpret_cast<u8*>(malloc(1024 * 1024 * 8)); // todo: replace with required_mem_size

	memory_provider mp;
	mp.persistent_memory = mem;
	mp.persistent_size = 1024 * 1024 * 6;
	mp.transient_memory = mem + (1024 * 1024 * 6);
	mp.transient_size = 1024 * 1024 * 2;

	if (xp_init(&mp, 64, 128))
	{
		xp_step(1.0 / 60.0);
		xp_destroy_body(xp_create_dynamic_body(10.0));
	}

	free(mem);

	return 0;
}

