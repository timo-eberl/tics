set(TARGET_NAME inelastic_impulse)

add_executable(${TARGET_NAME}
	main.c
)

target_link_libraries(${TARGET_NAME} PRIVATE tics)
