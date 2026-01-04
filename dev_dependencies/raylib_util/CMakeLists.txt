project(raylib_util)

add_library(${PROJECT_NAME} STATIC raylib_util.c)

target_include_directories(${PROJECT_NAME} PUBLIC .)

# PUBLIC means: "Anyone who uses this bridge also needs raylib"
target_link_libraries(${PROJECT_NAME} PUBLIC raylib)
