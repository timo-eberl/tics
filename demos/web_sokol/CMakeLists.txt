set(TARGET_NAME web_sokol)

add_executable(${TARGET_NAME} main.c)

target_include_directories(${TARGET_NAME} PRIVATE dependencies/sokol)

target_link_libraries(${TARGET_NAME} PRIVATE tics)

if(EMSCRIPTEN)
	# Tell Emscripten to use WebGL2 (required by Sokol GLES3 backend) and allow memory growth
	target_link_options(${TARGET_NAME} PRIVATE
		"-sUSE_WEBGL2=1"
		"-sMIN_WEBGL_VERSION=2"
		"-sMAX_WEBGL_VERSION=2"
		"-sALLOW_MEMORY_GROWTH=1"
		"-sEXPORTED_RUNTIME_METHODS=['UTF8ToString']"
		"--shell-file" "${CMAKE_CURRENT_SOURCE_DIR}/shell.html"
	)

	# Change the output from a command-line .js file to a full .html page
	set_target_properties(${TARGET_NAME} PROPERTIES SUFFIX ".html")
else()
	message(WARNING
		"Can't build web example natively"
	)
endif()
