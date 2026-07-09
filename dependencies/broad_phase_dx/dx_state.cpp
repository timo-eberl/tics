#include <cstdio>
#include <cstdlib>

#ifdef _WIN32
	// Not sure if this works, I only tested WSL so far
	#include <windows.h>
	#include <unknwn.h>
#else
	#include <wsl/winadapter.h>
#endif

#include <directx/d3d12.h>
#include <directx/dxcore.h>
#include <dxguids/dxguids.h>

extern "C" bool dx12_test_init(void) {
	IDXCoreAdapterFactory* factory = nullptr;
	HRESULT hr = DXCoreCreateAdapterFactory(IID_PPV_ARGS(&factory));
	if (FAILED(hr)) {
		std::fprintf(stderr, "Failed to create DXCore Adapter Factory.\n");
		return false;
	}

	IDXCoreAdapterList* adapter_list = nullptr;
	const GUID dx12_guid = DXCORE_ADAPTER_ATTRIBUTE_D3D12_GRAPHICS;
	hr = factory->CreateAdapterList(1, &dx12_guid, IID_PPV_ARGS(&adapter_list));
	if (FAILED(hr)) {
		std::fprintf(stderr, "Failed to create DXCore Adapter List.\n");
		factory->Release();
		return false;
	}

	IDXCoreAdapter* adapter = nullptr;
	ID3D12Device* device = nullptr;
	bool success = false;

	if (adapter_list->GetAdapterCount() > 0) {
		hr = adapter_list->GetAdapter(0, IID_PPV_ARGS(&adapter));
		if (SUCCEEDED(hr)) {
			hr = D3D12CreateDevice(adapter, D3D_FEATURE_LEVEL_12_0, IID_PPV_ARGS(&device));
			if (SUCCEEDED(hr)) {
				// Allocate buffer dynamically to bypass any terminal bracket stripping bugs
				size_t buf_size = 128;
				char* driver_desc = (char*)std::malloc(buf_size);

				if (driver_desc) {
					// GetProperty requires the scoped DXCoreAdapterProperty namespace
					hr = adapter->GetProperty(
						DXCoreAdapterProperty::DriverDescription,
						buf_size,
						driver_desc
					);
					if (SUCCEEDED(hr)) {
						std::printf("Successfully initialized D3D12 Device on: %s\n",
							driver_desc);
					} else {
						std::printf("Successfully initialized D3D12 Device.\n");
					}
					std::free(driver_desc);
				}
				success = true;
			} else {
				std::fprintf(stderr, "D3D12CreateDevice failed.\n");
			}
		}
	} else {
		std::fprintf(stderr, "No DX12 compatible adapters found.\n");
	}

	if (device) device->Release();
	if (adapter) adapter->Release();
	if (adapter_list) adapter_list->Release();
	if (factory) factory->Release();

	return success;
}
