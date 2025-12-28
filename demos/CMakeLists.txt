# Ron-the-renderer (own graphics library)
set(RON_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # dont build examples
# absolute path pointing to in demos/assets/
# Note: For deploying/installing, this path needs to be changed to a relative path
set(RON_ASSET_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/assets/) 
add_subdirectory(dependencies/ron-the-renderer)

# Add demo projects
add_subdirectory(playground)
add_subdirectory(raycasting)
