set(TARGET_NAME test_debug_view)

set(SOURCES
	main.c
)

add_executable(${TARGET_NAME} ${SOURCES})

target_include_directories(${TARGET_NAME} PRIVATE .)
# use private includes for tics_internal.h so we can setup a fake physics simulation
target_include_directories(${TARGET_NAME} PRIVATE ../../src)

target_link_libraries(${TARGET_NAME} PRIVATE tics)
