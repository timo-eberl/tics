# Disable raylib's extra stuff so it builds faster
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# EXCLUDE_FROM_ALL: Build lazy (don't build unless someone links to it)

add_subdirectory(cgltf-1.15 EXCLUDE_FROM_ALL)
add_subdirectory(pcg EXCLUDE_FROM_ALL)

# Unfortunately the configure step still always happens. For raylib/sokol this
# fails if dependencies are not found. To avoid this we check manually if they
# are needed by the debug viewer or the demos.
if(TICS_ENABLE_DEBUG_VIEW)
	add_subdirectory(raylib-5.5 EXCLUDE_FROM_ALL)
endif()

if(TICS_BUILD_DEMOS)
	add_subdirectory(sokol_util EXCLUDE_FROM_ALL)
endif()

if(TICS_BUILD_TOOL_GLTF2C)
	add_subdirectory(gltf2c)
endif()
