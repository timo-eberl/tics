set(TARGET_NAME spinning_in_vacuum)

add_executable(${TARGET_NAME}
	main.c
)

target_link_libraries(${TARGET_NAME} PRIVATE tics)
