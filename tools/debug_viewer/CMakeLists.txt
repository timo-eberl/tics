set(TARGET_NAME tics_debug_viewer)

add_executable(${TARGET_NAME}
	main.c
)

target_link_libraries(${TARGET_NAME} PRIVATE tics)
target_link_libraries(${TARGET_NAME} PRIVATE raylib)
target_link_libraries(${TARGET_NAME} PRIVATE tics_raylib_bridge)
