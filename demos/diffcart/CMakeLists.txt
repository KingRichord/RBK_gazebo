cmake_minimum_required(VERSION 2.8.3)
find_package(Eigen3 REQUIRED)
add_library(diffcart_plugin SHARED diffcart_plugin.cc)
target_link_libraries(diffcart_plugin gazebo_demos_proto communication ${GAZEBO_libraries})
add_dependencies(diffcart_plugin gazebo_demos_proto)


