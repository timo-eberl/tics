set(TARGET_NAME playground)

add_executable(${TARGET_NAME}
	main.c
)

target_link_libraries(${TARGET_NAME} PRIVATE tics)
target_link_libraries(${TARGET_NAME} PRIVATE raylib)
target_link_libraries(${TARGET_NAME} PRIVATE tics_raylib_bridge)
