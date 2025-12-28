# Ron-the-renderer (own graphics library)
set(RON_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # dont build examples
# In Dev: Project require a symlink next to the binary to point to the actual binary
# In Prod: The assets folder should be placed next to the binary
set(RON_ASSET_DIRECTORY "./assets/")
add_subdirectory(dependencies/ron-the-renderer)

# Add demo projects
add_subdirectory(playground)
add_subdirectory(raycasting)
