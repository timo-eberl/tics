cmake_minimum_required(VERSION 3.15)
project(broad_phase_dx LANGUAGES C CXX)

# Automatically download the DirectX Headers for Linux/WSL
include(FetchContent)
FetchContent_Declare(
	DirectX-Headers
	GIT_REPOSITORY https://github.com/microsoft/DirectX-Headers.git
	GIT_TAG        v1.619.4 # Most recent stable release as of 2025-07-09
)
FetchContent_MakeAvailable(DirectX-Headers)

# Look for DXC in PATH first
find_program(DXC_EXECUTABLE dxc)

if(NOT DXC_EXECUTABLE)
	message(STATUS "dxc not found in PATH. Automatically downloading prebuilt Microsoft DirectXShaderCompiler binary...")
	if(WIN32)
		set(DXC_URL "https://github.com/microsoft/DirectXShaderCompiler/releases/download/v1.9.2602.24/dxc_2026_05_27.zip ")
		set(DXC_HASH "SHA256=cf658aacf070d3045e31b8f1f8a696c2945f37c1095019481ef7c513368db3b4")
		set(DXC_BIN_RELATIVE "bin/x64/dxc.exe")
	else()
		# Assuming Linux/WSL x86_64
		set(DXC_URL "https://github.com/microsoft/DirectXShaderCompiler/releases/download/v1.9.2602.24/linux_dxc_2026_05_26.x86_64.tar.gz")
		set(DXC_HASH "SHA256=928b3e9986d11dc4279050e02340950c29bcbd1e5efb9d3ded9669dade37639d")
		set(DXC_BIN_RELATIVE "bin/dxc")
	endif()

	FetchContent_Declare(
		dxc_prebuilt
		URL ${DXC_URL}
		URL_HASH ${DXC_HASH}
	)
	FetchContent_GetProperties(dxc_prebuilt)
	if(NOT dxc_prebuilt_POPULATED)
		FetchContent_Populate(dxc_prebuilt)
	endif()
	set(DXC_EXECUTABLE "${dxc_prebuilt_SOURCE_DIR}/${DXC_BIN_RELATIVE}")

	if(UNIX)
		execute_process(COMMAND chmod +x "${DXC_EXECUTABLE}")
	endif()
endif()

message(STATUS "Using DXC: ${DXC_EXECUTABLE}")

add_custom_command(
	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/brute_force_shader.h
	COMMAND ${DXC_EXECUTABLE}
		-E main
		-T cs_6_0
		-O3
		-Vn brute_force_shader
		-Fh ${CMAKE_CURRENT_BINARY_DIR}/brute_force_shader.h
		${CMAKE_CURRENT_SOURCE_DIR}/brute_force.hlsl
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/brute_force.hlsl
)

add_library(${PROJECT_NAME} STATIC
	shared_state.cpp
	algo_brute_force.cpp
	${CMAKE_CURRENT_BINARY_DIR}/brute_force_shader.h
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

if(UNIX)
	# WSL: dxcore is used instead of dxgi
	target_link_directories(${PROJECT_NAME} PUBLIC "/usr/lib/wsl/lib")
	find_library(D3D12_LIB d3d12 PATHS "/usr/lib/wsl/lib" REQUIRED)
	find_library(DXCORE_LIB dxcore PATHS "/usr/lib/wsl/lib" REQUIRED)

	target_link_libraries(${PROJECT_NAME} PUBLIC 
		DirectX-Headers
		DirectX-Guids
		${D3D12_LIB}
		${DXCORE_LIB}
	)
else()
	# Native Windows
	target_link_libraries(${PROJECT_NAME} PUBLIC
		DirectX-Headers
		DirectX-Guids
		d3d12
		dxgi
	)
endif()
