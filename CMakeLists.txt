cmake_minimum_required(VERSION 3.7)
project(ukf_generate_sigmaPoints)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp ukf.cpp ukf.h)
add_executable(ukf_generate_sigmaPoints ${SOURCE_FILES})