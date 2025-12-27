cmake_minimum_required(VERSION 3.5)
project(physics-playground)
set (CMAKE_CXX_STANDARD 20) # Require C++20

# this triggers the use of the geometric algebra version
add_compile_definitions(TICS_GA)

# Generate compile_commands.json to make C++ syntax checkers happy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# dependencies
	# tics (own physics library)
	add_subdirectory (tics)

	# ron (own graphics library)
	set(RON_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # dont build examples
	set(RON_ASSET_DIRECTORY ${PROJECT_SOURCE_DIR}/assets/) # use our own assets directory
	# set(RON_ASSET_DIRECTORY ./assets/) # for exporting
	add_subdirectory (ron-the-renderer)

# this Project
	set(SOURCES
		src/game_loop.cpp
	)
	add_executable(${PROJECT_NAME}
		src/apps/playground.cpp
		# src/apps/raycasting.cpp
		${SOURCES}
	)
	target_include_directories(${PROJECT_NAME} PRIVATE src)
	# link libraries
	target_link_libraries(${PROJECT_NAME} PRIVATE tics)
	target_link_libraries(${PROJECT_NAME} PRIVATE ron)
