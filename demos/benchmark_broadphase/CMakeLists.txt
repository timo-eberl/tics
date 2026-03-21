set(TARGET_NAME benchmark_broadphase)

add_executable(${TARGET_NAME}
	main.c
)

target_link_libraries(${TARGET_NAME} PRIVATE tics)
target_link_libraries(${TARGET_NAME} PRIVATE pcg)
