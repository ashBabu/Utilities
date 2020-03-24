cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
project(eigenTest)

add_executable(
	eigenT
	eigen.cpp
  	# source1.cpp
  	# source2.cpp
	)