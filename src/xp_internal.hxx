#pragma once
#include "xp.h"

struct XPConvexHull
{
	usize nverts;
	real* verts; // interleaved xyz positions
};

struct OrientedBox
{
	real half_extents[3];
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

//	bool ContainsAddress(void* address) const { return (address > memory) && (address < (memory + capacity)); }

	// iterators
	T& operator[](usize i) { return memory[i]; }
	const T& operator[](usize i) const { return memory[i]; }
};

template <typename T>
struct MemoryRange
{
	T* head;
	T* tail;

	usize GetSize() const { return (u8*)tail - (u8*)head; }
	usize GetCount() const { return tail - head; }

	// iterators
	T& operator[](usize i) { return head[i]; }
	const T& operator[](usize i) const { return head[i]; }
};
