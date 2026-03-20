# Disable raylib's extra stuff so it builds faster
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# EXCLUDE_FROM_ALL: Build lazy (don't build unless someone links to it)

add_subdirectory(cgltf-1.15 EXCLUDE_FROM_ALL)
add_subdirectory(pcg EXCLUDE_FROM_ALL)

# Unfortunately the configure step still happens and fails if raylib
# dependencies are not found. So we still need to manually check if demos
# or debug viewer is built.
if(NOT EMSCRIPTEN AND (TICS_BUILD_DEMOS OR TICS_ENABLE_DEBUG_VIEW))
	add_subdirectory(raylib-5.5 EXCLUDE_FROM_ALL)
	add_subdirectory(tics_raylib_bridge EXCLUDE_FROM_ALL)
	add_subdirectory(raylib_util EXCLUDE_FROM_ALL)
endif()

if(TICS_BUILD_TOOL_GLTF2C)
	add_subdirectory(gltf2c)
endif()
