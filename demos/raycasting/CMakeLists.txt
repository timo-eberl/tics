set(TARGET_NAME raycasting)

set(SOURCES
	main.cpp
	game_loop.cpp
)

add_executable(${TARGET_NAME} ${SOURCES})

# Include current directory to find all header files in there
target_include_directories(${TARGET_NAME} PRIVATE .)

# Link libraries
target_link_libraries(${TARGET_NAME} PRIVATE tics)
target_link_libraries(${TARGET_NAME} PRIVATE ron)
