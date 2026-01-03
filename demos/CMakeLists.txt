# Disable raylib's extra stuff so it builds faster
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

add_subdirectory(dependencies/raylib-5.5)
add_subdirectory(dependencies/cgltf-1.15)

# Add demo projects
add_subdirectory(playground)
add_subdirectory(dummy_live_viewer)
