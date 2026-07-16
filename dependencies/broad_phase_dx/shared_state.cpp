#include "broad_phase_dx.h"
#include "dx_common.h"
#include "dx_profile.h"

extern "C" dx_shared_state* dx_shared_state_create(void) {
	dx_shared_state* s = (dx_shared_state*)calloc(1, sizeof(dx_shared_state));
	
	IDXCoreAdapterFactory* factory = nullptr;
	HRESULT hr = DXCoreCreateAdapterFactory(IID_PPV_ARGS(&factory));
	if (FAILED(hr)) {
		fprintf(stderr, "[dx12] Failed to create DXCore Adapter Factory.\n");
		return s;
	}

	IDXCoreAdapterList* adapter_list = nullptr;
	const GUID dx12_guid = DXCORE_ADAPTER_ATTRIBUTE_D3D12_GRAPHICS;
	hr = factory->CreateAdapterList(1, &dx12_guid, IID_PPV_ARGS(&adapter_list));
	if (FAILED(hr)) {
		fprintf(stderr, "[dx12] Failed to create DXCore Adapter List.\n");
		factory->Release();
		return s;
	}

	IDXCoreAdapter* adapter = nullptr;
	if (adapter_list->GetAdapterCount() > 0) {
		hr = adapter_list->GetAdapter(1, IID_PPV_ARGS(&adapter));
		if (SUCCEEDED(hr)) {
			ID3D12Device* base_device = nullptr;
			hr = D3D12CreateDevice(adapter, D3D_FEATURE_LEVEL_12_0, IID_PPV_ARGS(&base_device));
			
			if (SUCCEEDED(hr)) {
				// Query the ID3D12Device10 interface to access Enhanced Barriers Create API
				hr = base_device->QueryInterface(IID_PPV_ARGS(&s->device));
				base_device->Release();
				
				if (SUCCEEDED(hr)) {
					char desc[128] = {0};
					hr = adapter->GetProperty(DXCoreAdapterProperty::DriverDescription,
											  sizeof(desc), desc);
					if (SUCCEEDED(hr)) {
						fprintf(stderr, "[dx12] Initialized D3D12 Device on: %s\n", desc);
					} else {
						fprintf(stderr, "[dx12] Initialized D3D12 Device on unknown adapter.\n");
					}
				}
			}
		}
	} else {
		fprintf(stderr, "[dx12] No DX12 compatible adapters found.\n");
	}
	
	if (adapter) adapter->Release();
	if (adapter_list) adapter_list->Release();
	if (factory) factory->Release();

	if (!s->device) {
		fprintf(stderr, "[dx12] Failed to create D3D12 Device10.\n");
		return s;
	}

	D3D12_COMMAND_QUEUE_DESC q_desc = {};
	q_desc.Type = D3D12_COMMAND_LIST_TYPE_COMPUTE;
	q_desc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_NORMAL;
	q_desc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
	q_desc.NodeMask = 0;
	DX_CHECK(s->device->CreateCommandQueue(&q_desc, IID_PPV_ARGS(&s->cmd_queue)));

	DX_CHECK(s->device->CreateCommandAllocator(
		D3D12_COMMAND_LIST_TYPE_COMPUTE, IID_PPV_ARGS(&s->cmd_allocator)));

	DX_CHECK(s->device->CreateCommandList(
		0, D3D12_COMMAND_LIST_TYPE_COMPUTE, s->cmd_allocator, nullptr,
		IID_PPV_ARGS(&s->cmd_list)));

	// Setup Profiling Heaps
	DX_CHECK(s->cmd_queue->GetTimestampFrequency(&s->timestamp_frequency));
	D3D12_QUERY_HEAP_DESC qh_desc = {};
	qh_desc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
	qh_desc.Count = 32;
	qh_desc.NodeMask = 0;
	DX_CHECK(s->device->CreateQueryHeap(&qh_desc, IID_PPV_ARGS(&s->query_heap)));
	ensure_dx_buffer(s->device, &s->rb_query, &s->rb_query_size, 32, sizeof(uint64_t),
					 D3D12_HEAP_TYPE_READBACK, D3D12_RESOURCE_FLAG_NONE, 1.0f);

	DX_CHECK(s->device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&s->fence)));
	s->fence_value = 0;
	s->fence_event = dx_create_event();

	// Allocate a persistent 4-byte buffer containing 0 to quickly reset atomic counters
	ensure_dx_buffer(s->device, &s->up_zero, &s->up_zero_size, 1, sizeof(uint32_t),
					 D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_FLAG_NONE, 1.0f);
	void* p_zero;
	D3D12_RANGE read_range = {0, 0};
	s->up_zero->Map(0, &read_range, &p_zero);
	*(uint32_t*)p_zero = 0;
	s->up_zero->Unmap(0, nullptr);
	
	return s;
}

extern "C" void dx_shared_state_destroy(dx_shared_state* s) {
	if (!s) return;
	
	if (s->cmd_queue && s->fence && s->fence_event) {
		s->fence_value++;
		s->cmd_queue->Signal(s->fence, s->fence_value);
		if (s->fence->GetCompletedValue() < s->fence_value) {
			s->fence->SetEventOnCompletion(s->fence_value, (HANDLE)s->fence_event);
			dx_wait_event(s->fence_event);
		}
	}

	if (s->up_rigids) s->up_rigids->Release();
	if (s->up_statics) s->up_statics->Release();
	if (s->d_rigids) s->d_rigids->Release();
	if (s->d_statics) s->d_statics->Release();
	if (s->d_pairs) s->d_pairs->Release();
	if (s->d_pair_count) s->d_pair_count->Release();
	if (s->rb_pairs) s->rb_pairs->Release();
	if (s->rb_pair_count) s->rb_pair_count->Release();

	if (s->rb_query) s->rb_query->Release();
	if (s->query_heap) s->query_heap->Release();
	if (s->up_zero) s->up_zero->Release();

	if (s->fence_event) dx_close_event(s->fence_event);
	if (s->fence) s->fence->Release();
	if (s->cmd_list) s->cmd_list->Release();
	if (s->cmd_allocator) s->cmd_allocator->Release();
	if (s->cmd_queue) s->cmd_queue->Release();
	if (s->device) s->device->Release();
	
	free(s);
}

extern "C" void dx_shared_begin_pass(dx_shared_state* sh, const dx_aabb* rigids, int rigid_count,
									 const dx_aabb* statics, int static_count, bool statics_changed,
									 size_t pairs_needed, dx_profile* prof) {
	ensure_dx_buffer(sh->device, &sh->up_rigids, &sh->up_rigids_size, rigid_count,
					 sizeof(dx_aabb), D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_FLAG_NONE, 1.0f);
	ensure_dx_buffer(sh->device, &sh->d_rigids, &sh->d_rigids_size, rigid_count,
					 sizeof(dx_aabb), D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_FLAG_NONE, 1.0f);

	void* mapped = nullptr;
	D3D12_RANGE read_range = {0, 0};
	sh->up_rigids->Map(0, &read_range, &mapped);
	memcpy(mapped, rigids, rigid_count * sizeof(dx_aabb));
	sh->up_rigids->Unmap(0, nullptr);

	if (statics_changed && static_count > 0) {
		ensure_dx_buffer(sh->device, &sh->up_statics, &sh->up_statics_size, static_count,
						 sizeof(dx_aabb), D3D12_HEAP_TYPE_UPLOAD,
						 D3D12_RESOURCE_FLAG_NONE, 1.0f);
		ensure_dx_buffer(sh->device, &sh->d_statics, &sh->d_statics_size, static_count,
						 sizeof(dx_aabb), D3D12_HEAP_TYPE_DEFAULT,
						 D3D12_RESOURCE_FLAG_NONE, 1.0f);
						
		sh->up_statics->Map(0, &read_range, &mapped);
		memcpy(mapped, statics, static_count * sizeof(dx_aabb));
		sh->up_statics->Unmap(0, nullptr);
	}

	ensure_dx_buffer(sh->device, &sh->rb_pair_count, &sh->rb_pair_count_size, 1,
					 sizeof(uint32_t), D3D12_HEAP_TYPE_READBACK,
					 D3D12_RESOURCE_FLAG_NONE, 1.0f);
	ensure_dx_buffer(sh->device, &sh->d_pair_count, &sh->d_pair_count_size, 1,
					 sizeof(uint32_t), D3D12_HEAP_TYPE_DEFAULT,
					 D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, 1.0f);
	ensure_dx_buffer(sh->device, &sh->d_pairs, &sh->d_pairs_size, pairs_needed,
					 sizeof(dx_pair), D3D12_HEAP_TYPE_DEFAULT,
					 D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, 1.0f);

	dx_profile_begin(prof, sh);

	// Resources implicitly start at COMMON access allowing direct copy execution
	sh->cmd_list->CopyBufferRegion(sh->d_rigids, 0, sh->up_rigids, 0,
								   rigid_count * sizeof(dx_aabb));
	if (statics_changed && static_count > 0) {
		sh->cmd_list->CopyBufferRegion(sh->d_statics, 0, sh->up_statics, 0,
									   static_count * sizeof(dx_aabb));
	}
	sh->cmd_list->CopyBufferRegion(sh->d_pair_count, 0, sh->up_zero, 0, sizeof(uint32_t));

	dx_profile_step(prof, sh, "upload");

	// Global barrier to transition all buffers from the upload copy phase to compute phase.
	// We use a global barrier because it efficiently flushes all caches for the entire queue
	// without needing to track and transition individual buffer states.
	D3D12_GLOBAL_BARRIER global_barrier = {};
	global_barrier.SyncBefore = D3D12_BARRIER_SYNC_COPY;
	global_barrier.SyncAfter = D3D12_BARRIER_SYNC_COMPUTE_SHADING;
	global_barrier.AccessBefore = D3D12_BARRIER_ACCESS_COPY_DEST;
	global_barrier.AccessAfter = D3D12_BARRIER_ACCESS_SHADER_RESOURCE |
								 D3D12_BARRIER_ACCESS_UNORDERED_ACCESS;
	
	D3D12_BARRIER_GROUP barrier_group = {};
	barrier_group.Type = D3D12_BARRIER_TYPE_GLOBAL;
	barrier_group.NumBarriers = 1;
	barrier_group.pGlobalBarriers = &global_barrier;
	
	sh->cmd_list->Barrier(1, &barrier_group);
}

extern "C" uint32_t dx_shared_execute_and_get_count(dx_shared_state* sh, int static_count,
													dx_profile* prof) {
	// Wait for the compute shader to finish its UAV writes and flush the caches,
	// making the memory visible to the copy engine for readback.
	D3D12_GLOBAL_BARRIER global_barrier = {};
	global_barrier.SyncBefore = D3D12_BARRIER_SYNC_COMPUTE_SHADING;
	global_barrier.SyncAfter = D3D12_BARRIER_SYNC_COPY;
	global_barrier.AccessBefore = D3D12_BARRIER_ACCESS_UNORDERED_ACCESS;
	global_barrier.AccessAfter = D3D12_BARRIER_ACCESS_COPY_SOURCE;
	
	D3D12_BARRIER_GROUP barrier_group = {};
	barrier_group.Type = D3D12_BARRIER_TYPE_GLOBAL;
	barrier_group.NumBarriers = 1;
	barrier_group.pGlobalBarriers = &global_barrier;
	
	sh->cmd_list->Barrier(1, &barrier_group);

	sh->cmd_list->CopyBufferRegion(sh->rb_pair_count, 0, sh->d_pair_count, 0, sizeof(uint32_t));

	dx_profile_resolve(prof, sh);
	dx_execute_and_wait(sh);

	uint32_t count = 0;
	void* mapped = nullptr;
	D3D12_RANGE read_range = {0, 0};
	sh->rb_pair_count->Map(0, &read_range, &mapped);
	count = *(uint32_t*)mapped;
	sh->rb_pair_count->Unmap(0, nullptr);

	return count;
}

extern "C" dx_pair* dx_shared_readback_pairs(dx_shared_state* sh, uint32_t count,
											 dx_profile* prof) {
	ensure_dx_buffer(sh->device, &sh->rb_pairs, &sh->rb_pairs_size, count, sizeof(dx_pair),
					 D3D12_HEAP_TYPE_READBACK, D3D12_RESOURCE_FLAG_NONE, 2.0f);

	dx_profile_split(prof, sh);
	
	// ExecuteCommandLists guarantees all caches are flushed. Buffers inherently return
	// to COMMON access at the start of a command list, so no explicit transition is needed.
	sh->cmd_list->CopyBufferRegion(sh->rb_pairs, 0, sh->d_pairs, 0, count * sizeof(dx_pair));

	dx_profile_step(prof, sh, "readback");
	dx_profile_resolve(prof, sh);

	dx_execute_and_wait(sh);

	dx_pair* h_pairs = (dx_pair*)malloc(count * sizeof(dx_pair));
	void* mapped = nullptr;
	D3D12_RANGE read_range = {0, 0};
	sh->rb_pairs->Map(0, &read_range, &mapped);
	memcpy(h_pairs, mapped, count * sizeof(dx_pair));
	sh->rb_pairs->Unmap(0, nullptr);

	return h_pairs;
}
