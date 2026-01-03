set(TARGET_NAME falling_cube)

set(SOURCES
	main.c
)

add_executable(${TARGET_NAME} ${SOURCES})

target_include_directories(${TARGET_NAME} PRIVATE .)

target_link_libraries(${TARGET_NAME} PRIVATE tics)
