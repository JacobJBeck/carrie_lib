project(MULTIAGENT)
cmake_minimum_required(VERSION 2.8)
include_directories(include ../../src)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../../build")
file(GLOB MULTIAGENT_SRC
	"src/*.cpp"
)

file(GLOB MULTIAGENT_INCLUDE
	"include/*.h"
)

add_library(${PROJECT_NAME} ${MULTIAGENT_SRC} ${MULTIAGENT_INCLUDE})
