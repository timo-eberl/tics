cmake_minimum_required(VERSION 3.10)
project(tics)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON) # We need gnu11 for stb_ds.h

# Options to enable/disable demos/tests/...
# By default everything will be built, but can be disabled like this in a CMakeLists.txt:
#   set(TICS_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
# or when running cmake:
#   cmake -DTICS_BUILD_DEMOS=OFF

option(TICS_BUILD_DEMOS "Build demo applications" ON)
option(TICS_BUILD_TESTS "Build tests" OFF)
option(TICS_ENABLE_DEBUG_VIEW "Enable the visual debugger (unix only)" ON)
option(TICS_BUILD_TOOL_GLB2C "Build glb model to C source code conversion tool" OFF)

if(TICS_ENABLE_DEBUG_VIEW)
	if(NOT UNIX)
		message(WARNING 
			"TICS_ENABLE_DEBUG_VIEW was requested, but the current platform is not Unix-like.\n"
			"The visual debugger relies on POSIX Shared Memory and Forking.\n"
			"--> Automatically disabling TICS_ENABLE_DEBUG_VIEW for this build."
		)
		set(TICS_ENABLE_DEBUG_VIEW OFF CACHE BOOL "Enable the external visual debugger process" FORCE)
	endif()
endif()

# Enable LTO/IPO only for Release configuration
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# We put all executables (for demos, tests, debug visualization) in the same folder (build/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Dependencies
	# stb_ds.h (C library for dynamic arrays and hash tables)
	add_subdirectory(dependencies/stb_ds)

# Our Library Project
set(SOURCES
	src/core.c
	src/step.c
	src/collision_test.c
	src/tics_debug_view_shm.c
)

add_library(${PROJECT_NAME} ${SOURCES})

target_compile_options(${PROJECT_NAME} PRIVATE
	$<$<AND:$<CONFIG:Release>,$<C_COMPILER_ID:GNU,Clang>>:-march=native>
)

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PRIVATE src)

target_link_libraries(${PROJECT_NAME} PRIVATE stb_ds)
target_link_libraries(${PROJECT_NAME} PRIVATE m) # math library

if(TICS_BUILD_DEMOS)
	add_subdirectory(demos)
endif()

if(TICS_BUILD_TESTS)
	add_subdirectory(tests)
endif()

if(TICS_ENABLE_DEBUG_VIEW)
	target_compile_definitions(tics PUBLIC TICS_ENABLE_DEBUG_VIEW)

	# Link Realtime (rt) and Pthread libraries for Shared Memory on Linux/Unix
	if(UNIX)
		find_package(Threads REQUIRED)
		# Link Realtime (rt) for shared memory and Threads
		target_link_libraries(tics PRIVATE rt Threads::Threads)
	endif()

	add_subdirectory(tools/debug_viewer)
endif()

if(TICS_BUILD_TOOL_GLB2C)
	add_subdirectory(tools/glb2c)
endif()
