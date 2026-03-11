#pragma once
#include "xp.hxx"

struct xp_convex_hull
{
	usize nverts;
	real* verts; // interleaved xyz positions
};

template <typename T>
struct linear_allocator
{
	T* memory;
	usize size;
	usize offset;

	linear_allocator() : memory(nullptr), size(0), offset(0) {}
	linear_allocator(T* memory, usize size) : memory(memory), size(size), offset(0) {}

	void* push_bytes(usize count)
	{
		if (offset + count > size)
			return nullptr;

		void* result = reinterpret_cast<u8*>(memory) + offset;
		offset += count;
		return result;
	}

	void reset() { offset = 0; }

	T& operator[](usize i) { return memory[i]; }
	const T& operator[](usize i) const { return memory[i]; }
};

template <typename T>
struct memory_range
{
	T* begin;
	T* end;

	usize size() const { return end - begin; }
	T& operator[](usize i) { return begin[i]; }
	const T& operator[](usize i) const { return begin[i]; }
};
