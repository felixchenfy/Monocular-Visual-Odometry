add_library( myslam SHARED
    config.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)

add_executable(main main.cpp)
target_link_libraries( main
    myslam
)
