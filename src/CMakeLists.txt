add_library(rby1a
        integrated_robot.cpp
)
target_include_directories(rby1a PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(rby1a PUBLIC
        rby1-sdk::rby1-sdk
        ${OpenCV_LIBRARIES}
)
target_link_libraries(rby1a PRIVATE
        tomlplusplus::tomlplusplus
        realsense2
)