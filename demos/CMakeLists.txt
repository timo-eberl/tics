# Disable raylib's extra stuff so it builds faster
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

add_subdirectory(dependencies/raylib-5.5)

# Add demo projects
add_subdirectory(playground)
add_subdirectory(test_debug_view)
add_subdirectory(falling_cube)
