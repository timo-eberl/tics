# Ron-the-renderer (own graphics library)
set(RON_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # dont build examples
# In Dev: Project requires a symlink next to the binary to point to the actual assets dir
# In Prod: The assets dir should be placed next to the binary
set(RON_ASSET_DIRECTORY ./assets/)
# Set RON_ASSET_COPY_DIRECTORY to absolute path
# This tells Ron to put its default assets into the actual assets dir during build time.
set(RON_ASSET_COPY_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/assets/")
add_subdirectory(dependencies/ron-the-renderer)

# Add demo projects
add_subdirectory(playground)
add_subdirectory(raycasting)
