cmake_minimum_required(VERSION 3.9)
project(test_lib)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB sources *.cpp)

add_library(SUBPROBLEMS ${sources})
