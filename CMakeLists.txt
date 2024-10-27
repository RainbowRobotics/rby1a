cmake_minimum_required(VERSION 3.28)
project(rby1a_data_collection)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(third-party)

find_package(rby1-sdk REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(src)
add_subdirectory(examples)