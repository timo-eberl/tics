set(TARGET_NAME frame_dump_renderer)

add_executable(${TARGET_NAME}
	main.c
)

target_link_libraries(${TARGET_NAME} PRIVATE blick)
target_link_libraries(${TARGET_NAME} PRIVATE m)

# For development. Allows us to load the collision_test_data.bin from the project root, regardless
# of the working directory
target_compile_definitions(${TARGET_NAME} PRIVATE PROJECT_ROOT_DIR=\"${PROJECT_SOURCE_DIR}\")
