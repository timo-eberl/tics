project(cgltf)

add_library(${PROJECT_NAME} STATIC src/cgltf.c)
target_include_directories(${PROJECT_NAME} PUBLIC include)
