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

add_library(${PROJECT_NAME} STATIC
	dx_state.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

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

# Note on Shaders (HLSL):
# Since DX12 compute shaders must be written in HLSL, you will likely need to 
# compile them using the DirectX Shader Compiler (dxc).
# You can compile them at build time into C headers using a custom command like:
#
# add_custom_command(
# 	OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/compute_shader.h
# 	COMMAND dxc -E main -T cs_6_0 -Fh ${CMAKE_CURRENT_BINARY_DIR}/compute_shader.h ${CMAKE_CURRENT_SOURCE_DIR}/compute.hlsl
# 	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/compute.hlsl
# )
# target_sources(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/compute_shader.h)
# target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
