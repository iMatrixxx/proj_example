cmake_minimum_required(VERSION 3.10)
project(proj_example)
 
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -g -Wextra -Wpedantic)
include_directories(.)
include_directories(proj)
 
find_package(PROJ)
 find_package(Eigen3 REQUIRED)




add_executable(proj_example example.cpp)
target_include_directories(proj_example
 PRIVATE
 ${EIGEN3_INCLUDE_DIRS}

)
target_link_libraries(proj_example ${PROJ_LIBRARIES})