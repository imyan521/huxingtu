cmake_minimum_required(VERSION 3.16)
project(floorplan_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_library(floorplan STATIC
        floor_plan.cpp
)
target_include_directories(floorplan PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(floorplan PUBLIC ${OpenCV_LIBS})

add_executable(end_to_end_floor_plan
        end_to_end_floor_plan.cpp
)
target_link_libraries(end_to_end_floor_plan PRIVATE floorplan)
