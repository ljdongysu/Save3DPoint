cmake_minimum_required(VERSION 3.20)
project(Save3DPoint)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#add_library(GetCamera)
#target_link_libraries(GetCamera ${OpenCV_LIBS} -ldl)

add_executable(Save3DPoint  main.cpp point_cloud.cpp point_cloud.h)
target_link_libraries (Save3DPoint ${OpenCV_LIBRARIES})
