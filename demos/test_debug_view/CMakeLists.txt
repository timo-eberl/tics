set(TARGET_NAME test_debug_view)

add_executable(${TARGET_NAME}
	main.c
)

# use private includes for tics_internal.h so we can setup a fake physics simulation
target_include_directories(${TARGET_NAME} PRIVATE ../../src)

target_link_libraries(${TARGET_NAME} PRIVATE tics)
