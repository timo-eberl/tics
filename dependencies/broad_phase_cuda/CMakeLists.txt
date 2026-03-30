cmake_minimum_required(VERSION 3.18) # 3.18+ has native CUDA_ARCHITECTURES support

# This sub-project uses CUDA — only these files are compiled with nvcc
project(broad_phase_cuda LANGUAGES CUDA C)

# find_package is needed to get the CUDA::cudart imported target
find_package(CUDAToolkit REQUIRED)

add_library(${PROJECT_NAME} STATIC
	shared_state.cu
	algo_brute_force.cu
	algo_grid_a.cu
	algo_grid_b.cu
)

# Expose benchmark configuration to the CMake command line
set(GRID_CELL_SIZE "1.0f" CACHE STRING "Size of the uniform grid cells")
set(GRID_RES_X "100" CACHE STRING "Grid resolution in the x dimension")
set(GRID_RES_Y "100" CACHE STRING "Grid resolution in the y dimension")
set(GRID_RES_Z "100" CACHE STRING "Grid resolution in the z dimension")
target_compile_definitions(${PROJECT_NAME} PRIVATE
	GRID_CELL_SIZE=${GRID_CELL_SIZE}
	GRID_RES_X=${GRID_RES_X}
	GRID_RES_Y=${GRID_RES_Y}
	GRID_RES_Z=${GRID_RES_Z}
)

# Required for viewing debug stats for individual code lines
target_compile_options(${PROJECT_NAME} PRIVATE -lineinfo)

# Command to gather profiling data (for all kernels starting with grid_ into file profile_output):
# sudo "$(which ncu)" --set full --kernel-name-base demangled --kernel-name 'regex:^grid_.*' -o profile_output ./popcorn_machine

# Expose the pure-C header so the main tics library can include it
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME} PUBLIC 
	CUDA::cudart
	stdc++ # link with C++ runtime required by CUDA
)

# Set CUDA standard
set_target_properties(${PROJECT_NAME} PROPERTIES
	CUDA_STANDARD 17
	CUDA_STANDARD_REQUIRED ON
	POSITION_INDEPENDENT_CODE ON
	CUDA_ARCHITECTURES "native" # Compile only for the installed GPU
)
