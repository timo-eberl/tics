cmake_minimum_required(VERSION 3.18)  # 3.18+ has native CUDA_ARCHITECTURES support

# This sub-project uses CUDA — only these files are compiled with nvcc
project(tics_cuda LANGUAGES CUDA C)

add_library(tics_cuda STATIC broad_phase_cuda.cu)

# Expose the pure-C header so the main tics library can include it
target_include_directories(tics_cuda PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Directly link against the absolute path to libcudart.so
# because nothing else worked (Hopefully I can replace this stupid CUDA stuff
# soon with OpenCL)
target_link_libraries(tics_cuda PUBLIC 
	/usr/local/cuda-13.1/lib64/libcudart.so
    stdc++ # link with C++ runtime required by CUDA
)

# Set CUDA standard
set_target_properties(tics_cuda PROPERTIES
	CUDA_STANDARD 11
	CUDA_STANDARD_REQUIRED ON
	POSITION_INDEPENDENT_CODE ON
	# Common choices:
	#   52  = Maxwell (GTX 900 series)
	#   61  = Pascal  (GTX 1000 series)
	#   75  = Turing  (RTX 2000 series)
	#   86  = Ampere  (RTX 3000 series)
	#   89  = Ada     (RTX 4000 series)
	CUDA_ARCHITECTURES "75;86;89"
)
