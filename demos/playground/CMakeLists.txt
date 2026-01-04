set(TARGET_NAME playground)

set(SOURCES
	main.c
	demo_framework.c
)

add_executable(${TARGET_NAME} ${SOURCES})

# Include current directory to find all header files in there
target_include_directories(${TARGET_NAME} PRIVATE .)

# Link libraries
target_link_libraries(${TARGET_NAME} PRIVATE tics)
target_link_libraries(${TARGET_NAME} PRIVATE raylib)
target_link_libraries(${TARGET_NAME} PRIVATE demos_cgltf)

# Create a symlink "./assets" in the build dir to actual assets dir
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E create_symlink
	"${CMAKE_CURRENT_SOURCE_DIR}/../assets"      # actual assets dir
	"$<TARGET_FILE_DIR:${TARGET_NAME}>/assets"   # link next to the binary
	COMMENT "Symlinking assets to build directory"
)
