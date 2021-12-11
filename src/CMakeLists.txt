cmake_minimum_required(VERSION 3.21.2)

project(main CXX)

add_subdirectory(lib)
add_subdirectory(planning)

add_executable(main main/main.cpp)
target_link_libraries(main planning)