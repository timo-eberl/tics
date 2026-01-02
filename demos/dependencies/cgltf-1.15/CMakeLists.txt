cmake_minimum_required(VERSION 3.5...3.30)
project(cgltf)

add_library(${PROJECT_NAME} src/cgltf.c)
target_include_directories(${PROJECT_NAME} PUBLIC include)
