cmake_minimum_required(VERSION 3.5...3.30)
project(stb_ds)

add_library(${PROJECT_NAME} src/stb_ds.c)
target_include_directories(${PROJECT_NAME} PUBLIC include)
