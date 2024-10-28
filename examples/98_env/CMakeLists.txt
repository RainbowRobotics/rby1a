add_executable(env
        main.cpp
)
target_link_libraries(env PRIVATE
        rby1a
        libzmq
        cppzmq
)