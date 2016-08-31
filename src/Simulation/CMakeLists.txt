project(SIMULATION)
cmake_minimum_required(VERSION 2.8)
include_directories(include ../../src)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../../build")
file(GLOB SIMULATION_SRC
	"src/*.cpp"
)

file(GLOB SIMULATION_INCLUDE
	"include/*.h"
)

add_library(${PROJECT_NAME} ${SIMULATION_SRC} ${SIMULATION_INCLUDE})
