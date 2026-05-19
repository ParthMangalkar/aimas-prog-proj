cmake_minimum_required(VERSION 3.16)
project(searchclient_cpp_v2 CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

add_executable(searchclient_cpp_v2
    src/main.cpp
    src/core.cpp
    src/parser.cpp
    src/topology.cpp
    src/single_box.cpp
    src/pibt.cpp
    src/compact.cpp
    src/solver.cpp
)

target_include_directories(searchclient_cpp_v2 PRIVATE include)

enable_testing()
add_executable(v2_tests
    tests/test_main.cpp
    src/core.cpp
    src/parser.cpp
    src/topology.cpp
    src/single_box.cpp
    src/pibt.cpp
    src/compact.cpp
    src/solver.cpp
)
target_include_directories(v2_tests PRIVATE include)
add_test(NAME v2_tests COMMAND v2_tests)
