cmake_minimum_required(VERSION 3.16)
project(RBK_gazebo)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
add_compile_options(-std=c++17)


find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_subdirectory(demos)
add_subdirectory(communication)

add_executable(RBK_gazebo main.cpp)
target_link_libraries(RBK_gazebo  ${GAZEBO_LIBRARIES})


