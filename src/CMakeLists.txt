cmake_minimum_required(VERSION 3.10)

project(Ocean_Simualtion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Stonefish REQUIRED 1.3.0)

add_executable(OceanApp main.cpp ocean_sim.cpp)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(/usr/local/include/Stonefish)

link_directories(/usr/local/lib)

target_link_libraries(OceanApp Stonefish::Stonefish)