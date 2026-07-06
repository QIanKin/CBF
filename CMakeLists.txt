cmake_minimum_required(VERSION 3.16)
project(cbf_controller LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

add_library(cbf_controller cbf_controller.cpp cbf_controller.hpp)
target_include_directories(cbf_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(cbf_controller PUBLIC Eigen3::Eigen OsqpEigen::OsqpEigen)