add_executable(98_env
        main.cpp
)
target_link_libraries(98_env PRIVATE
        rby1a
        libzmq
        cppzmq
        HDF5::HDF5
        HighFive
)