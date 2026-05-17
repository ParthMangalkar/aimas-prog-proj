cmake_minimum_required(VERSION 3.16)
project(searchclient_cpp LANGUAGES CXX)

add_executable(searchclient_cpp main.cpp)
target_compile_features(searchclient_cpp PRIVATE cxx_std_17)

add_library(mapf_experiments STATIC
    mapf/ALNS.cpp
    mapf/ALNSDestroy.cpp
    mapf/ALNSRepair.cpp
    mapf/BFSDistanceMap.cpp
    mapf/CAAStar.cpp
    mapf/ECBS.cpp
    mapf/ReservationTable.cpp
    mapf/SpaceTimeAStar.cpp
    mapf/TaskAllocator.cpp
)
target_compile_features(mapf_experiments PRIVATE cxx_std_17)

if (MSVC)
    target_compile_options(searchclient_cpp PRIVATE /W4)
    target_compile_options(mapf_experiments PRIVATE /W4)
else()
    target_compile_options(searchclient_cpp PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(mapf_experiments PRIVATE -Wall -Wextra -Wpedantic)
endif()

if (WIN32)
    target_link_libraries(searchclient_cpp PRIVATE psapi)
endif()
