cmake_minimum_required(VERSION 3.15)
project(tics)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON) # We need gnu11 for stb_ds.h

# --- Define Build Options ---

# Options to enable or disable demos, tests, tools,...
# By default everything will be built, but can be disabled like this in a CMakeLists.txt:
#   set(TICS_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
# or when running cmake:
#   cmake -DTICS_BUILD_DEMOS=OFF

option(TICS_BUILD_DEMOS "Build demo applications" ON)
option(TICS_BUILD_TESTS "Build tests" ON)
option(TICS_ENABLE_DEBUG_VIEW "Enable the visual debugger (unix only)" ON)
option(TICS_BUILD_TOOL_GLB2C "Build tool glb2c (converts glb model to C source code)" ON)

if(TICS_ENABLE_DEBUG_VIEW)
	if(NOT UNIX)
		message(WARNING 
			"TICS_ENABLE_DEBUG_VIEW was requested, but the current platform is not Unix-like.\n"
			"The visual debugger relies on POSIX Shared Memory and Forking.\n"
			"--> Automatically disabling TICS_ENABLE_DEBUG_VIEW for this build."
		)
		set(TICS_ENABLE_DEBUG_VIEW OFF CACHE BOOL "" FORCE)
	endif()
endif()

# Enable LTO/IPO only for Release configuration
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- The Tics library ---

# Dependency: stb_ds.h (C library for dynamic arrays and hash tables)
add_subdirectory(dependencies/stb_ds)

set(SOURCES
	src/core.c
	src/step.c
	src/collision_test.c
	src/tics_debug_view_shm.c
)
add_library(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PRIVATE src)

target_link_libraries(${PROJECT_NAME} PRIVATE tics_stb_ds)
target_link_libraries(${PROJECT_NAME} PRIVATE m) # math system library

# This is bad for portability, but for now it's fine
target_compile_options(${PROJECT_NAME} PRIVATE
	$<$<AND:$<CONFIG:Release>,$<C_COMPILER_ID:GNU,Clang>>:-march=native>
)

# --- Add Dev Dependencies ---

# add this first, because consumers (demos, tests, tools,...) need the library targets to exist
# before they can link them
add_subdirectory(dev_dependencies)

# --- Add Consumers ---

# We put all consumer executables in the same folder (build/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

if(TICS_BUILD_DEMOS)
	add_subdirectory(demos)
endif()

if(TICS_BUILD_TESTS)
	add_subdirectory(tests)
endif()

if(TICS_ENABLE_DEBUG_VIEW)
	# If this flag is set, we are guaranteed to be on unix

	target_compile_definitions(${PROJECT_NAME} PUBLIC TICS_ENABLE_DEBUG_VIEW)

	# Link Realtime (rt) and Pthread libraries for Shared Memory on Linux/Unix
	find_package(Threads REQUIRED)
	# Link Realtime (rt) for shared memory and Threads
	target_link_libraries(${PROJECT_NAME} PRIVATE rt Threads::Threads)

	add_subdirectory(tools/debug_viewer)
endif()

if(TICS_BUILD_TOOL_GLB2C)
	add_subdirectory(tools/glb2c)
endif()
