set(TARGET_NAME dummy_live_viewer)

set(SOURCES
	main.c
)

add_executable(${TARGET_NAME} ${SOURCES})

# Include current directory to find all header files in there
target_include_directories(${TARGET_NAME} PRIVATE .)
target_include_directories(${TARGET_NAME} PRIVATE ../../src)

# Link libraries
target_link_libraries(${TARGET_NAME} PRIVATE tics)
