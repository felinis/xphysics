//
//	xphysics rigid body dynamics engine
//	(C) 2026 Alan Moczulski
//

#include "xp.hxx"
#include <new>

namespace xp
{
	template <typename T>
	class linear_allocator
	{
		T* memory;
		usize size;
		usize offset;

	public:
		linear_allocator() : memory(nullptr), size(0), offset(0) {}
		linear_allocator(T* memory, usize size) : memory(memory), size(size), offset(0) {}

		T* allocate(usize count)
		{
			if (offset + count > size)
				return nullptr;

			T* result = memory + offset;
			offset += count;
			return result;
		}

		void deallocate(T* memory, usize count) {}

		void reset() { offset = 0; }
	};

	struct memory_context
	{
		linear_allocator<u8> persistent_allocator;
		linear_allocator<u8> transient_allocator;
	};

	class instance_impl final : public instance
	{
		memory_context memctx;

	public:
		instance_impl(const memory_context& memctx) : memctx(memctx) {}

		virtual void step(second dt) override
		{
			
		}

		virtual id create_fixed_body() override
		{
			return 0;
		}

		virtual id create_dynamic_body(kilogram mass) override
		{
			return 0;
		}

		virtual void destroy_body(id body_id) override
		{
			
		}

		virtual id create_convex_shape(const real* vertex_positions, u32 vertex_count) override
		{
			return 0;
		}

		virtual void attach_shape(id body_id, id shape_id) override
		{
			
		}

		virtual void get_body_position(id body_id, real out_position[3]) const override
		{
			
		}

		virtual void set_body_position(id body_id, const real position[3]) override
		{
			
		}
	};
}

XP_EXTERN_C XP_API xp::instance* xp_create_instance(const xp::memory_provider& provider)
{
	if (!provider.persistent_memory || !provider.transient_memory)
		return nullptr;

	xp::memory_context memctx;
	memctx.persistent_allocator = xp::linear_allocator<xp::u8>(provider.persistent_memory, provider.persistent_size);
	memctx.transient_allocator = xp::linear_allocator<xp::u8>(provider.transient_memory, provider.transient_size);

	// put this instance class in persistent memory and instantiate it
	void* memory = memctx.persistent_allocator.allocate(sizeof(xp::instance_impl));
	return new (memory) xp::instance_impl(memctx);
}
