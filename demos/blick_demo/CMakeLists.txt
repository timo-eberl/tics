set(TARGET_NAME blick_demo)

add_executable(${TARGET_NAME}
	main.c
)

target_link_libraries(${TARGET_NAME} PRIVATE blick)
