#include "xp.h"
#include <stdlib.h>

int main()
{
	xp_context_create_info xpcci;
	xpcci.persistent_memory = malloc(sizeof(void*) * 1024 * 1024);
	xpcci.persistent_memory_size = sizeof(void*) * 1024 * 1024;
	xpcci.transient_memory = malloc(sizeof(void*) * 128 * 1024);
	xpcci.transient_memory_size = sizeof(void*) * 128 * 1024;

	xp_context context = xp_init(&xpcci);
	xp_shutdown(context);

	free(xpcci.persistent_memory);
	free(xpcci.transient_memory);

	return 0;
}

