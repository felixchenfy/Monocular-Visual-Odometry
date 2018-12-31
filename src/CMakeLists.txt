add_library( myslam SHARED
    config.cpp
    visual_odometry.cpp
    camera.cpp
    frame.cpp
    # map.cpp
    # mappoint.cpp
    feature_match.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)

add_executable(main main.cpp)
target_link_libraries( main
    myslam
)
