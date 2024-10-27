add_subdirectory(third-party)

add_executable(data_collector
        main.cpp
        app_main.cpp
)
target_link_libraries(data_collector PRIVATE
        rby1a
)
target_link_libraries(data_collector PRIVATE
        tomlplusplus::tomlplusplus
        libzmq
        cppzmq
        nlohmann_json
)