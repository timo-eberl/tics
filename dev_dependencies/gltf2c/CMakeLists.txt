set(TARGET_NAME gltf2c)

add_executable(${TARGET_NAME} main.c)

target_link_libraries(${TARGET_NAME} PRIVATE cgltf)
target_link_libraries(${TARGET_NAME} PRIVATE m) # math system library
