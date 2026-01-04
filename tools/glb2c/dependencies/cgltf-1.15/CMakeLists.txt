project(cgltf)

add_library(${PROJECT_NAME} src/cgltf.c)
target_include_directories(${PROJECT_NAME} PUBLIC include)
