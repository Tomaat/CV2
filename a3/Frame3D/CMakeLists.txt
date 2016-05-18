cmake_minimum_required(VERSION 2.8.9)
project(frame3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.45.0 REQUIRED COMPONENTS filesystem system)

set(OpenCV_STATIC ON)
set(OpenCV_CUDA OFF)
find_package( OpenCV REQUIRED )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

file(GLOB SOURCES "src/*.cpp")

add_executable(frame3d ${SOURCES})

target_link_libraries(frame3d ${Boost_LIBRARIES} ${OpenCV_LIBS})