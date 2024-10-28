find_package(HDF5 REQUIRED)
add_subdirectory(third-party)

add_executable(data_collector
        main.cpp
        app_main.cpp
)
target_include_directories(data_collector PUBLIC
        ${HDF5_INCLUDE_DIR}
)
target_link_libraries(data_collector PRIVATE
        rby1a
        tomlplusplus::tomlplusplus
        libzmq
        cppzmq
        nlohmann_json
        HDF5::HDF5
        HighFive
)