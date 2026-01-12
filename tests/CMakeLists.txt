set(TEST_SOURCES
	test_main.c
	test_api.c
	test_collision_test.c
)

add_executable(test_runner ${TEST_SOURCES})

target_link_libraries(test_runner PRIVATE
	tics
	blick
)

# Include 'include' (Public API) and 'src' (Internal/White-box testing)
target_include_directories(test_runner PRIVATE 
	${CMAKE_SOURCE_DIR}/include 
	${CMAKE_SOURCE_DIR}/src
)
