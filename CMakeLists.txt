cmake_minimum_required(VERSION 3.16)
project(Test_assignment)

set(CMAKE_CXX_STANDARD 17)
include_directories(includes/Eigen)
include_directories(headers)

add_executable(Test_assignment src/main.cpp
        src/key_points.cc headers/key_points.h
        src/task_solver.cc headers/task_solver.h)