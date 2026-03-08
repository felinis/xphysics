#include "xp.hxx"
#include <stdlib.h>

int main()
{
	xp::memory_provider provider;
	provider.persistent_memory = reinterpret_cast<xp::u8*>(malloc(sizeof(void*) * 1024 * 1024));
	provider.persistent_size = sizeof(void*) * 1024 * 1024;
	provider.transient_memory = reinterpret_cast<xp::u8*>(malloc(sizeof(void*) * 128 * 1024));
	provider.transient_size = sizeof(void*) * 128 * 1024;

	xp::instance* instance = xp_create_instance(provider);
	instance->step(1.0 / 60.0);

	instance->destroy_body(instance->create_dynamic_body(1.0));

	free(provider.persistent_memory);
	free(provider.transient_memory);

	return 0;
}

