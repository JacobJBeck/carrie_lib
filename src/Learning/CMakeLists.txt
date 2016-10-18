project(LEARNING)
cmake_minimum_required(VERSION 2.8)
include_directories(include ../../src)
set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive -g ${CMAKE_CXX_FLAGS}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../../build")
file(GLOB LEARNING_SRC
	"src/*.cpp"
)

file(GLOB LEARNING_INCLUDE
	"include/*.h"
)

add_library(${PROJECT_NAME} ${LEARNING_SRC} ${LEARNING_INCLUDE})
