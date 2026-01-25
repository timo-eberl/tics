set(TEST_SOURCES
	entry.c
	test_api.c
	test_collision_test.c
	test_dynamics.c
	test_velocity_at_point.c
)

add_executable(test_runner ${TEST_SOURCES})

target_link_libraries(test_runner PRIVATE tics)

if(TICS_ENABLE_DEBUG_VIEW)
	target_link_libraries(test_runner PRIVATE blick)
endif()

# Include 'include' (Public API) and 'src' (Internal/White-box testing)
target_include_directories(test_runner PRIVATE 
	${CMAKE_SOURCE_DIR}/include 
	${CMAKE_SOURCE_DIR}/src
)
