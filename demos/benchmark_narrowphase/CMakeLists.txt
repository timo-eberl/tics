set(TARGET_NAME benchmark_narrowphase)

add_executable(${TARGET_NAME}
	main.c
)

# Expose benchmark configuration to the CMake command line
set(BENCHMARK_STEPS "1200" CACHE STRING "Number of steps for the benchmark")
set(BENCHMARK_PARTICLE_COUNT "15000" CACHE STRING "Number of particles in the benchmark scene")
target_compile_definitions(${TARGET_NAME} PRIVATE
	BENCHMARK_STEPS=${BENCHMARK_STEPS}
	BENCHMARK_PARTICLE_COUNT=${BENCHMARK_PARTICLE_COUNT}
)

target_link_libraries(${TARGET_NAME} PRIVATE tics)
target_link_libraries(${TARGET_NAME} PRIVATE pcg)
