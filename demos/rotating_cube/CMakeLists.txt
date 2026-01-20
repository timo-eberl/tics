set(TARGET_NAME rotating_cube)

add_executable(${TARGET_NAME}
	main.c
)

target_link_libraries(${TARGET_NAME} PRIVATE tics)
