project(PLANNING)
cmake_minimum_required(VERSION 2.8)
include_directories(include ../../src)
set(CMAKE_CXX_FLAGS "-std=c++1y ${CMAKE_CXX_FLAGS}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../../build")
file(GLOB PLANNING_SRC
	"src/*.cpp"
)

file(GLOB PLANNING_INCLUDE
	"include/*.h"
)

add_library(${PROJECT_NAME} ${PLANNING_SRC} ${PLANNING_INCLUDE})
