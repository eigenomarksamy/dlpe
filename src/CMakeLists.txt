cmake_minimum_required(VERSION 3.21.2)

project(main CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# enable repository specific options
include("${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ProjectSpecificOptions.cmake")

add_subdirectory(lib)
add_subdirectory(planning)

add_executable(main main/main.cpp)
target_link_libraries(main planning)