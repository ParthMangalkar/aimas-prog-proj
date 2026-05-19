cmake_minimum_required(VERSION 3.16)
project(searchclient_cpp_enhanced LANGUAGES CXX)

add_executable(searchclient_cpp_enhanced src/main.cpp)
target_compile_features(searchclient_cpp_enhanced PRIVATE cxx_std_17)

if (MSVC)
    target_compile_options(searchclient_cpp_enhanced PRIVATE /W4)
else()
    target_compile_options(searchclient_cpp_enhanced PRIVATE -Wall -Wextra -Wpedantic)
endif()

