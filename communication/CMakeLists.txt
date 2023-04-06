cmake_minimum_required(VERSION 3.0.0)
project(communication)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")
find_package(Threads)
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

SET(OpenCV_DIR ${RBK_THIRDPARTY_LIB_DIR}/opencv)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_PATH})
set(ZeroMQ_DIR /home/moi/APP/include/zmq/lib/cmake/ZeroMQ)
find_package(ZeroMQ REQUIRED)
file(GLOB_RECURSE ALL_SRC
        **.cpp
        **.h)
add_library(communication  ${ALL_SRC})

target_include_directories(communication PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/sonic/sonic
        ${EIGEN_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${ZeroMQ_INCLUDE_DIRS}
        )
## Specify libraries to link a library or executable target against
target_link_libraries(communication
        ${OpenCV_LIBS}
        ${ZeroMQ_LIBRARIES}
)