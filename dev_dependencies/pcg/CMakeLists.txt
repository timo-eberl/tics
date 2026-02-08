project(pcg)

add_library(${PROJECT_NAME} STATIC pcg.c)
target_include_directories(${PROJECT_NAME} PUBLIC .)
