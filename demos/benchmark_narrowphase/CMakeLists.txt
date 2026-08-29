set(TARGET_NAME benchmark_narrowphase)

add_executable(${TARGET_NAME}
	main.c
)

# Expose benchmark configuration to the CMake command line
set(NARROW_BENCHMARK_STEPS "1200" CACHE STRING "Number of steps for the benchmark")
set(NARROW_BENCHMARK_SPHERE_COUNT "5000" CACHE STRING "Number of spheres in the benchmark scene")
set(NARROW_BENCHMARK_CAPSULE_COUNT "5000" CACHE STRING "Number of capsules in the benchmark scene")
set(NARROW_BENCHMARK_BOX_COUNT "5000" CACHE STRING "Number of boxes in the benchmark scene")
target_compile_definitions(${TARGET_NAME} PRIVATE
	BENCHMARK_STEPS=${NARROW_BENCHMARK_STEPS}
	BENCHMARK_SPHERE_COUNT=${NARROW_BENCHMARK_SPHERE_COUNT}
	BENCHMARK_CAPSULE_COUNT=${NARROW_BENCHMARK_CAPSULE_COUNT}
	BENCHMARK_BOX_COUNT=${NARROW_BENCHMARK_BOX_COUNT}
)

target_link_libraries(${TARGET_NAME} PRIVATE tics)
target_link_libraries(${TARGET_NAME} PRIVATE pcg)
