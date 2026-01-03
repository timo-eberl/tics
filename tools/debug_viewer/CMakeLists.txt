set(TARGET_NAME tics_debug_viewer)

set(SOURCES
	main.c
)

add_executable(${TARGET_NAME} ${SOURCES})

target_link_libraries(${TARGET_NAME} PRIVATE tics)
