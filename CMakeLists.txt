cmake_minimum_required(VERSION 3.10)
project(proj_example)
 
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -g -Wextra -Wpedantic)
include_directories(.)
include_directories(proj)
 
find_package(PROJ)
 
add_executable(proj_example example.cpp)
target_link_libraries(proj_example ${PROJ_LIBRARIES})