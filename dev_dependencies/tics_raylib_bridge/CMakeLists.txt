project(tics_raylib_bridge)

add_library(${PROJECT_NAME} STATIC tics_raylib_bridge.c)

target_include_directories(${PROJECT_NAME} PUBLIC .)

# PUBLIC means: "Anyone who uses this bridge ALSO needs tics and raylib"
target_link_libraries(${PROJECT_NAME} PUBLIC
	tics
	raylib
)
