cmake_minimum_required(VERSION 3.21.2)

project(planning CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

# set include directories variables
set(INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/engine
    ${CMAKE_CURRENT_SOURCE_DIR}/grid_planning
)

# set source files variable
set(SOURCES_CPP
    ${CMAKE_CURRENT_SOURCE_DIR}/grid_planning/astar.cpp
)

add_library(planning STATIC ${SOURCES_CPP})

target_include_directories(planning PUBLIC ${INCLUDE_DIR})

target_link_libraries(planning utils)