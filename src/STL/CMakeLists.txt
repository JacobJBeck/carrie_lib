project(STL)
cmake_minimum_required(VERSION 2.8)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../../build")
file(GLOB STL_SRC
	"src/*.cpp"
)

file(GLOB STL_INCLUDE
	"include/*.h"
)

add_library(${PROJECT_NAME} ${STL_SRC} ${STL_INCLUDE})
