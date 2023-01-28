cmake_minimum_required(VERSION 2.6 FATAL_ERROR)   
project(test)      


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall ")
add_compile_options(-std=c++17)


find_package(PCL 1.11 REQUIRED)     
include_directories(${PCL_INCLUDE_DIRS})                
link_directories(${PCL_LIBRARY_DIRS})                   
add_definitions(${PCL_DEFINITIONS}) 

find_package(Boost REQUIRED COMPONENTS system chrono thread)
include_directories(${Boost_INCLUDE_DIRS})

add_executable(1-4PCS 1-4PCS.cpp)           
target_link_libraries(1-4PCS ${PCL_LIBRARIES} ${Boost_LIBRARIES}) 