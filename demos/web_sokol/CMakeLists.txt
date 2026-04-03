set(TARGET_NAME web_sokol)

add_executable(${TARGET_NAME} main.c)

target_include_directories(${TARGET_NAME} PRIVATE ../../dev_dependencies/sokol)

target_link_libraries(${TARGET_NAME} PRIVATE tics)

if(EMSCRIPTEN)
	# Tell Emscripten to use WebGL2 (required by Sokol GLES3 backend) and allow memory growth
	target_link_options(${TARGET_NAME} PRIVATE
		"-sMIN_WEBGL_VERSION=2"
		"-sMAX_WEBGL_VERSION=2"
		"-sALLOW_MEMORY_GROWTH=1"
		"--shell-file" "${CMAKE_CURRENT_SOURCE_DIR}/../../dev_dependencies/sokol_util/shell.html"
	)

	# Change the output from a command-line .js file to a full .html page
	set_target_properties(${TARGET_NAME} PROPERTIES SUFFIX ".html")
else()
	if(CMAKE_SYSTEM_NAME MATCHES "Linux")
		# Linux OpenGL & X11 dependencies
		target_link_libraries(${TARGET_NAME} PRIVATE
			X11 Xi Xcursor GL m
		)
	elseif(WIN32)
		# This project requires MinGW, which requires explicit D3D11 linking
		target_link_libraries(${TARGET_NAME} PRIVATE d3d11 dxgi)
	endif()
endif()
