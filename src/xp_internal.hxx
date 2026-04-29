#pragma once
#include "xp.h"

struct XPConvexHull
{
	usize nverts;
	real* verts; // interleaved xyz positions
};

template <typename T>
struct LinearAllocator
{
	T* memory;
	usize capacity;
	usize size;

	LinearAllocator() = default;
	explicit LinearAllocator(T* memory, usize capacity) : memory(memory), capacity(capacity), size(0) {}

	void* PushBytes(usize count)
	{
		// make sure we don't go out of bounds
		if (size + count > capacity)
			return nullptr;

		void* result = reinterpret_cast<u8*>(memory) + size;
		size += count;
		return result;
	}

	void Reset() { size = 0; }

	T& operator[](usize i) { return memory[i]; }
	const T& operator[](usize i) const { return memory[i]; }
};

template <typename T>
struct MemoryRange
{
	T* begin;
	T* end;

	usize GetSize() const { return end - begin; }
	usize GetCount() const { return (end - begin) / sizeof(T); }

	// iterators
	T& operator[](usize i) { return begin[i]; }
	const T& operator[](usize i) const { return begin[i]; }
};
