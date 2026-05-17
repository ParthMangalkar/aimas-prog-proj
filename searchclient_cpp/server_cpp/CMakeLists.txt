cmake_minimum_required(VERSION 3.16)
project(server_cpp LANGUAGES CXX)

add_executable(server_cpp main.cpp)
target_compile_features(server_cpp PRIVATE cxx_std_17)

if (MSVC)
    target_compile_options(server_cpp PRIVATE /W4)
else()
    target_compile_options(server_cpp PRIVATE -Wall -Wextra -Wpedantic)
endif()
