set(TARGET_NAME glb2c)

set(SOURCES
	main.c
)

add_subdirectory(dependencies/cgltf-1.15)

add_executable(${TARGET_NAME} ${SOURCES})

target_link_libraries(${TARGET_NAME} PRIVATE cgltf)
target_link_libraries(${TARGET_NAME} PRIVATE m) # math library
