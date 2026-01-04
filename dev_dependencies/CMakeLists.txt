# Disable raylib's extra stuff so it builds faster
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

# EXCLUDE_FROM_ALL: Build lazy (don't build unless someone links to it)

add_subdirectory(cgltf-1.15 EXCLUDE_FROM_ALL)
add_subdirectory(raylib-5.5 EXCLUDE_FROM_ALL)
add_subdirectory(tics_raylib_bridge EXCLUDE_FROM_ALL)
