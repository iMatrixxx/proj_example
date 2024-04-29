cmake_minimum_required(VERSION 3.10)
project(proj_example)
 
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -g -Wextra -Wpedantic)
include_directories(.)
include_directories(./include)
include_directories(proj)
 
find_package(PROJ)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


set(src_files
  "example.cpp"
  # "src/coordinate_trans.cpp"
)


add_executable(proj_example ${src_files})
target_include_directories(proj_example
 PRIVATE
 ${CMAKE_CURRENT_SOURCE_DIR}/include
 ${EIGEN3_INCLUDE_DIRS}

)
target_link_libraries(proj_example ${PROJ_LIBRARIES})

include_directories(
    ./
    ./include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

set(test_stitch_image_files
    "stictch_image/stitch_image.cpp"
)

add_executable(test_stitch_image ${test_stitch_image_files})
target_link_libraries(test_stitch_image ${OpenCV_LIBS})