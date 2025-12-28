cmake_minimum_required(VERSION 3.10)
project(tics)
set(CMAKE_CXX_STANDARD 20) # Require C++20

# Enable LTO/IPO only for Release configuration
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Our Library Project
set(SOURCES
	src/world.cpp
	src/collision_test.cpp
	src/impulse_solver.cpp
	src/position_solver.cpp
	src/static_body.cpp
	src/rigid_body.cpp
	src/collision_area.cpp
	src/collision_area_solver.cpp
	src/raycast.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC include)

# the demo program(s) will be built by default, they can be disabled like this:
# set(TICS_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
option(TICS_BUILD_DEMOS "Build demo applications" ON)

if(TICS_BUILD_DEMOS)
	add_subdirectory(demos)
endif()
