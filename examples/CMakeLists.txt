
add_subdirectory(third-party)

add_executable(example_robot
        robot.cpp
)
target_link_libraries(example_robot PRIVATE
        rby1a
)

add_executable(example_master_arm
        master_arm.cpp
)
target_link_libraries(example_master_arm PRIVATE
        rby1a
)

add_subdirectory(98_env)
add_subdirectory(99_data_collector)