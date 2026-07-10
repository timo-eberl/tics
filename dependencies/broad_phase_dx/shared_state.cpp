#include "broad_phase_dx.h"
#include "dx_common.h"

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

	if (s->fence_event) dx_close_event(s->fence_event);
	if (s->fence) s->fence->Release();
	if (s->cmd_list) s->cmd_list->Release();
	if (s->cmd_allocator) s->cmd_allocator->Release();
	if (s->cmd_queue) s->cmd_queue->Release();
	if (s->device) s->device->Release();
	
	free(s);
}
