project(blick)

# Require C11 for stdatomic.h support
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# POSIX Realtime (rt) and Threads (pthread) are required for Shared Memory and Atomics on Linux
find_library(LIB_RT rt)

if(WIN32)
	set(BLICK_OS_SRC src/blick_os_win32.c)
else()
	set(BLICK_OS_SRC src/blick_os_posix.c)
endif()

# --- Blick Static Library (The Host Side, linked to by an Application) ---
add_library(${PROJECT_NAME} STATIC
	src/blick.c
	${BLICK_OS_SRC}
)
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIB_RT} pthread)
target_include_directories(${PROJECT_NAME} PUBLIC include)

# --- Blick Viewer (The Client Side) ---
add_executable(${PROJECT_NAME}_viewer
	src/viewer_main.c
	${BLICK_OS_SRC}
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
