#ifndef DX_COMMON_H
#define DX_COMMON_H

#include "broad_phase_dx.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
	#include <windows.h>
#else
	#include <wsl/winadapter.h>
	#include <sys/eventfd.h>
	#include <unistd.h>
	#include <poll.h>
#endif

#include <directx/d3d12.h>
#include <directx/dxcore.h>
#include <dxguids/dxguids.h>

struct dx_shared_state {
	ID3D12Device10* device;
	ID3D12CommandQueue* cmd_queue;
	ID3D12CommandAllocator* cmd_allocator;
	ID3D12GraphicsCommandList7* cmd_list;
	
	ID3D12Fence* fence;
	uint64_t fence_value;
	void* fence_event;

	// Input Upload Buffers (CPU -> GPU)
	ID3D12Resource* up_rigids;
	size_t up_rigids_size;
	ID3D12Resource* up_statics;
	size_t up_statics_size;

	// Default Buffers (GPU Only)
	ID3D12Resource* d_rigids;
	size_t d_rigids_size;
	ID3D12Resource* d_statics;
	size_t d_statics_size;
	ID3D12Resource* d_pairs;
	size_t d_pairs_size;
	ID3D12Resource* d_pair_count;
	size_t d_pair_count_size;

	// Output Readback Buffers (GPU -> CPU)
	ID3D12Resource* rb_pairs;
	size_t rb_pairs_size;
	ID3D12Resource* rb_pair_count;
	size_t rb_pair_count_size;

	// Profiling
	uint64_t timestamp_frequency;
	ID3D12QueryHeap* query_heap;
	ID3D12Resource* rb_query;
	size_t rb_query_size;

	// Utility Buffers
	ID3D12Resource* up_zero;
	size_t up_zero_size;
};

#define DX_CHECK(call) \
	do { \
		HRESULT hr_ = (call); \
		if (FAILED(hr_)) { \
			fprintf(stderr, "[dx12] %s:%d HRESULT 0x%08X\n", __FILE__, __LINE__, \
					(unsigned int)hr_); \
		} \
	} while (0)

#ifdef _WIN32
	static inline void* dx_create_event(void) {
		return CreateEventA(nullptr, FALSE, FALSE, nullptr);
	}
	static inline void dx_close_event(void* handle) {
		if (handle) CloseHandle((HANDLE)handle);
	}
	static inline void dx_wait_event(void* handle) {
		WaitForSingleObject((HANDLE)handle, INFINITE);
	}
#else
	static inline void* dx_create_event(void) {
		int fd = eventfd(0, 0);
		if (fd < 0) return nullptr;
		return (void*)(uintptr_t)fd;
	}
	static inline void dx_close_event(void* handle) {
		int fd = (int)(uintptr_t)handle;
		if (fd >= 0) close(fd);
	}
	static inline void dx_wait_event(void* handle) {
		int fd = (int)(uintptr_t)handle;
		if (fd < 0) return;
		struct pollfd pfd = {};
		pfd.fd = fd;
		pfd.events = POLLIN;
		poll(&pfd, 1, -1);
		uint64_t val = 0;
		ssize_t r = read(fd, &val, sizeof(val));
		(void)r;
	}
#endif

static inline void ensure_dx_buffer(ID3D12Device10* device, ID3D12Resource** d_buf,
									size_t* capacity, size_t needed, size_t elem_size,
									D3D12_HEAP_TYPE heap_type, D3D12_RESOURCE_FLAGS flags,
									float growth_factor) {
	if (*capacity >= needed) return;
	if (*d_buf) {
		(*d_buf)->Release();
		*d_buf = nullptr;
	}
	
	size_t target_capacity = (size_t)(needed * growth_factor);
	if (target_capacity < needed) {
		target_capacity = needed;
	}
	*capacity = 0;

	D3D12_HEAP_PROPERTIES heap_props = {};
	heap_props.Type = heap_type;
	heap_props.CPUPageProperty = D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
	heap_props.MemoryPoolPreference = D3D12_MEMORY_POOL_UNKNOWN;
	heap_props.CreationNodeMask = 1;
	heap_props.VisibleNodeMask = 1;

	D3D12_RESOURCE_DESC1 desc = {};
	desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
	desc.Alignment = 0;
	desc.Width = target_capacity * elem_size;
	desc.Height = 1;
	desc.DepthOrArraySize = 1;
	desc.MipLevels = 1;
	desc.Format = DXGI_FORMAT_UNKNOWN;
	desc.SampleDesc.Count = 1;
	desc.SampleDesc.Quality = 0;
	desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
	desc.Flags = flags;

	// Buffers physically have no layout, so we use UNDEFINED.
	HRESULT hr = device->CreateCommittedResource3(
		&heap_props, D3D12_HEAP_FLAG_NONE, &desc, D3D12_BARRIER_LAYOUT_UNDEFINED,
		nullptr, nullptr, 0, nullptr, IID_PPV_ARGS(d_buf));

	if (SUCCEEDED(hr)) {
		*capacity = target_capacity;
	} else {
		fprintf(stderr, "[dx12] Failed to allocate buffer of size %zu\n",
				target_capacity * elem_size);
	}
}

static inline void dx_execute_and_wait(dx_shared_state* sh) {
	DX_CHECK(sh->cmd_list->Close());
	ID3D12CommandList* lists[] = { sh->cmd_list };
	sh->cmd_queue->ExecuteCommandLists(1, lists);
	
	sh->fence_value++;
	DX_CHECK(sh->cmd_queue->Signal(sh->fence, sh->fence_value));
	
	if (sh->fence->GetCompletedValue() < sh->fence_value) {
		DX_CHECK(sh->fence->SetEventOnCompletion(sh->fence_value, (HANDLE)sh->fence_event));
		dx_wait_event(sh->fence_event);
	}
	
	DX_CHECK(sh->cmd_allocator->Reset());
	DX_CHECK(sh->cmd_list->Reset(sh->cmd_allocator, nullptr));
}

#include "dx_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

// Allocates resources, uploads host data, and synchronizes memory for the compute execution.
void dx_shared_begin_pass(dx_shared_state* sh, const dx_aabb* rigids, int rigid_count,
						  const dx_aabb* statics, int static_count, bool statics_changed,
						  size_t pairs_needed, dx_profile* prof);

// Synchronizes the command queue after the compute phase, schedules readback of the atomic
// counter, flushes, and returns the total amount of pairs generated.
uint32_t dx_shared_execute_and_get_count(dx_shared_state* sh, int static_count, dx_profile* prof);

// Copies the output pairs into a readback heap, executes the queue, and returns the
// final malloc'd array of pairs to the host.
dx_pair* dx_shared_readback_pairs(dx_shared_state* sh, uint32_t count, dx_profile* prof);

#ifdef __cplusplus
}
#endif

#endif
