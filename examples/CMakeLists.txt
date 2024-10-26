add_executable(example_robot
    robot.cpp
)
target_link_libraries(example_robot PRIVATE
    rby1a
)