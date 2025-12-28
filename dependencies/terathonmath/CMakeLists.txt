cmake_minimum_required(VERSION 3.5)
project(terathonmath)
set (CMAKE_CXX_STANDARD 20) # not sure what C++ version this library actually uses

# Generate compile_commands.json to make C++ syntax checkers happy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SOURCES
	Terathon-Math-Library/TSAlgebra.cpp
	Terathon-Math-Library/TSBivector3D.cpp
	Terathon-Math-Library/TSConformal2D.cpp
	Terathon-Math-Library/TSConformal3D.cpp
	Terathon-Math-Library/TSFlector2D.cpp
	Terathon-Math-Library/TSFlector3D.cpp
	Terathon-Math-Library/TSMath.cpp
	Terathon-Math-Library/TSMatrix2D.cpp
	Terathon-Math-Library/TSMatrix3D.cpp
	Terathon-Math-Library/TSMatrix4D.cpp
	Terathon-Math-Library/TSMotor2D.cpp
	Terathon-Math-Library/TSMotor3D.cpp
	Terathon-Math-Library/TSQuaternion.cpp
	Terathon-Math-Library/TSRigid2D.cpp
	Terathon-Math-Library/TSRigid3D.cpp
	Terathon-Math-Library/TSVector2D.cpp
	Terathon-Math-Library/TSVector3D.cpp
	Terathon-Math-Library/TSVector4D.cpp
)
add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC Terathon-Math-Library)
