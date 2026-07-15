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
		hr = adapter_list->GetAdapter(0, IID_PPV_ARGS(&adapter));
		if (SUCCEEDED(hr)) {
			hr = D3D12CreateDevice(adapter, D3D_FEATURE_LEVEL_12_0, IID_PPV_ARGS(&s->device));
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
	} else {
		fprintf(stderr, "[dx12] No DX12 compatible adapters found.\n");
	}
	
	if (adapter) adapter->Release();
	if (adapter_list) adapter_list->Release();
	if (factory) factory->Release();

	if (!s->device) {
		fprintf(stderr, "[dx12] Failed to create D3D12 Device.\n");
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
					 D3D12_HEAP_TYPE_READBACK, D3D12_RESOURCE_STATE_COPY_DEST,
					 D3D12_RESOURCE_FLAG_NONE, 1.0f);

	DX_CHECK(s->device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&s->fence)));
	s->fence_value = 0;
	s->fence_event = dx_create_event();

	// Allocate a persistent 4-byte buffer containing 0 to quickly reset atomic counters
	ensure_dx_buffer(s->device, &s->up_zero, &s->up_zero_size, 1, sizeof(uint32_t),
					 D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ,
					 D3D12_RESOURCE_FLAG_NONE, 1.0f);
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
	// Allocate buffers
	ensure_dx_buffer(sh->device, &sh->up_rigids, &sh->up_rigids_size, rigid_count,
					 sizeof(dx_aabb), D3D12_HEAP_TYPE_UPLOAD,
					 D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_FLAG_NONE, 1.0f);
	ensure_dx_buffer(sh->device, &sh->d_rigids, &sh->d_rigids_size, rigid_count,
					 sizeof(dx_aabb), D3D12_HEAP_TYPE_DEFAULT,
					 D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_FLAG_NONE, 1.0f);

	void* mapped = nullptr;
	D3D12_RANGE read_range = {0, 0};
	sh->up_rigids->Map(0, &read_range, &mapped);
	memcpy(mapped, rigids, rigid_count * sizeof(dx_aabb));
	sh->up_rigids->Unmap(0, nullptr);

	if (statics_changed && static_count > 0) {
		ensure_dx_buffer(sh->device, &sh->up_statics, &sh->up_statics_size, static_count,
						 sizeof(dx_aabb), D3D12_HEAP_TYPE_UPLOAD,
						 D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_FLAG_NONE, 1.0f);
		ensure_dx_buffer(sh->device, &sh->d_statics, &sh->d_statics_size, static_count,
						 sizeof(dx_aabb), D3D12_HEAP_TYPE_DEFAULT,
						 D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_FLAG_NONE, 1.0f);
						 
		sh->up_statics->Map(0, &read_range, &mapped);
		memcpy(mapped, statics, static_count * sizeof(dx_aabb));
		sh->up_statics->Unmap(0, nullptr);
	}

	ensure_dx_buffer(sh->device, &sh->rb_pair_count, &sh->rb_pair_count_size, 1,
					 sizeof(uint32_t), D3D12_HEAP_TYPE_READBACK,
					 D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_FLAG_NONE, 1.0f);
	ensure_dx_buffer(sh->device, &sh->d_pair_count, &sh->d_pair_count_size, 1,
					 sizeof(uint32_t), D3D12_HEAP_TYPE_DEFAULT,
					 D3D12_RESOURCE_STATE_COMMON,
					 D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, 1.0f);
	ensure_dx_buffer(sh->device, &sh->d_pairs, &sh->d_pairs_size, pairs_needed,
					 sizeof(dx_pair), D3D12_HEAP_TYPE_DEFAULT,
					 D3D12_RESOURCE_STATE_COMMON,
					 D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, 1.0f);

	dx_profile_begin(prof, sh);

	D3D12_RESOURCE_BARRIER barriers[8] = {};
	int b_idx = 0;
	auto add_transition = [&](ID3D12Resource* res, D3D12_RESOURCE_STATES before,
							  D3D12_RESOURCE_STATES after) {
		if (before == after) return;
		barriers[b_idx].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
		barriers[b_idx].Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
		barriers[b_idx].Transition.pResource = res;
		barriers[b_idx].Transition.Subresource = 0;
		barriers[b_idx].Transition.StateBefore = before;
		barriers[b_idx].Transition.StateAfter = after;
		b_idx++;
	};

	add_transition(sh->d_rigids, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST);
	if (statics_changed && static_count > 0) {
		add_transition(sh->d_statics, D3D12_RESOURCE_STATE_COMMON,
					   D3D12_RESOURCE_STATE_COPY_DEST);
	}
	add_transition(sh->d_pair_count, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST);
	if (b_idx > 0) sh->cmd_list->ResourceBarrier(b_idx, barriers);

	sh->cmd_list->CopyBufferRegion(sh->d_rigids, 0, sh->up_rigids, 0,
								   rigid_count * sizeof(dx_aabb));
	if (statics_changed && static_count > 0) {
		sh->cmd_list->CopyBufferRegion(sh->d_statics, 0, sh->up_statics, 0,
									   static_count * sizeof(dx_aabb));
	}
	sh->cmd_list->CopyBufferRegion(sh->d_pair_count, 0, sh->up_zero, 0, sizeof(uint32_t));

	dx_profile_step(prof, sh, "upload");

	b_idx = 0;
	add_transition(sh->d_rigids, D3D12_RESOURCE_STATE_COPY_DEST,
				   D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
	if (statics_changed && static_count > 0) {
		add_transition(sh->d_statics, D3D12_RESOURCE_STATE_COPY_DEST,
					   D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
	} else if (static_count > 0) {
		add_transition(sh->d_statics, D3D12_RESOURCE_STATE_COMMON,
					   D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
	}
	add_transition(sh->d_pair_count, D3D12_RESOURCE_STATE_COPY_DEST,
				   D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
	add_transition(sh->d_pairs, D3D12_RESOURCE_STATE_COMMON,
				   D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
	if (b_idx > 0) sh->cmd_list->ResourceBarrier(b_idx, barriers);
}

extern "C" uint32_t dx_shared_execute_and_get_count(dx_shared_state* sh, int static_count,
													dx_profile* prof) {
	D3D12_RESOURCE_BARRIER barriers[8] = {};
	int b_idx = 0;
	auto add_transition = [&](ID3D12Resource* res, D3D12_RESOURCE_STATES before,
							  D3D12_RESOURCE_STATES after) {
		if (before == after) return;
		barriers[b_idx].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
		barriers[b_idx].Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;
		barriers[b_idx].Transition.pResource = res;
		barriers[b_idx].Transition.Subresource = 0;
		barriers[b_idx].Transition.StateBefore = before;
		barriers[b_idx].Transition.StateAfter = after;
		b_idx++;
	};

	add_transition(sh->d_pair_count, D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
				   D3D12_RESOURCE_STATE_COPY_SOURCE);
	if (b_idx > 0) sh->cmd_list->ResourceBarrier(b_idx, barriers);

	sh->cmd_list->CopyBufferRegion(sh->rb_pair_count, 0, sh->d_pair_count, 0, sizeof(uint32_t));

	b_idx = 0;
	add_transition(sh->d_rigids, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
				   D3D12_RESOURCE_STATE_COMMON);
	if (static_count > 0) {
		add_transition(sh->d_statics, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE,
					   D3D12_RESOURCE_STATE_COMMON);
	}
	add_transition(sh->d_pairs, D3D12_RESOURCE_STATE_UNORDERED_ACCESS,
				   D3D12_RESOURCE_STATE_COMMON);
	add_transition(sh->d_pair_count, D3D12_RESOURCE_STATE_COPY_SOURCE,
				   D3D12_RESOURCE_STATE_COMMON);
	if (b_idx > 0) sh->cmd_list->ResourceBarrier(b_idx, barriers);

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
					 D3D12_HEAP_TYPE_READBACK, D3D12_RESOURCE_STATE_COPY_DEST,
					 D3D12_RESOURCE_FLAG_NONE, 2.0f);

	dx_profile_split(prof, sh);
	
	D3D12_RESOURCE_BARRIER barriers[1] = {};
	barriers[0].Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
	barriers[0].Transition.pResource = sh->d_pairs;
	barriers[0].Transition.Subresource = 0;
	barriers[0].Transition.StateBefore = D3D12_RESOURCE_STATE_COMMON;
	barriers[0].Transition.StateAfter = D3D12_RESOURCE_STATE_COPY_SOURCE;
	sh->cmd_list->ResourceBarrier(1, barriers);

	sh->cmd_list->CopyBufferRegion(sh->rb_pairs, 0, sh->d_pairs, 0, count * sizeof(dx_pair));

	barriers[0].Transition.StateBefore = D3D12_RESOURCE_STATE_COPY_SOURCE;
	barriers[0].Transition.StateAfter = D3D12_RESOURCE_STATE_COMMON;
	sh->cmd_list->ResourceBarrier(1, barriers);

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
