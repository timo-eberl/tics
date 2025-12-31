cmake_minimum_required(VERSION 3.10)
project(tics)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON) # We need gnu11 for stb_ds.h

# Enable LTO/IPO only for Release configuration
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
	# stb_ds.h (C library for dynamic arrays and hash tables)
	add_subdirectory(dependencies/stb_ds)

# Our Library Project
set(SOURCES
	src/tics.c
)

add_library(${PROJECT_NAME} ${SOURCES})

target_compile_options(${PROJECT_NAME} PRIVATE
	$<$<AND:$<CONFIG:Release>,$<CXX_COMPILER_ID:GNU,Clang>>:-march=native>
)

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PRIVATE src)

target_link_libraries(${PROJECT_NAME} PRIVATE stb_ds)
target_link_libraries(${PROJECT_NAME} PRIVATE m) # math library

# the demo program(s) will be built by default, they can be disabled like this:
# set(TICS_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
option(TICS_BUILD_DEMOS "Build demo applications" ON)

if(TICS_BUILD_DEMOS)
	add_subdirectory(demos)
endif()

if(TICS_BUILD_TESTS)
	add_subdirectory(tests)
endif()
