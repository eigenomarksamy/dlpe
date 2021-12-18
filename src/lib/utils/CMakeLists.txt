project(utils)

cmake_minimum_required(VERSION 3.21.2)

add_library(utils)
target_include_directories(utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(SOURCES_CPP
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/printer.cpp
)

target_sources(utils PRIVATE ${SOURCES_CPP})
set_target_properties(utils
                      PROPERTIES
                      CXX_STANDARD 20
                      CXX_STANDARD_REQUIRED YES
                      CXX_EXTENSIONS NO
                      CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wall -Werror -g"
)
# target_link_libraries(utils PRIVATE project_options project_warnings)