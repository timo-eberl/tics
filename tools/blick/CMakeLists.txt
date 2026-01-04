project(blick)

# POSIX Realtime (rt) and Threads (pthread) are required for Shared Memory and Atomics on Linux
find_library(LIB_RT rt)

# --- Blick Static Library (The Host Side, linked to by an Application) ---
add_library(${PROJECT_NAME} STATIC
	src/blick.c
)
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIB_RT} pthread)
target_include_directories(${PROJECT_NAME} PUBLIC include)

# --- Blick Viewer (The Client Side) ---
add_executable(${PROJECT_NAME}_viewer
	src/viewer_main.c
)
target_include_directories(${PROJECT_NAME}_viewer PRIVATE include)
# Link viewer with raylib and system dependencies
target_link_libraries(${PROJECT_NAME}_viewer PRIVATE
	raylib
	raylib_util
	${LIB_RT}
	m
	pthread
)
