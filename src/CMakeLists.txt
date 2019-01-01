
add_library(mygeometry SHARED
    feature_match.cpp
    epipolar_geometry.cpp
    config.cpp
)
add_library( myslam SHARED
    config.cpp
    visual_odometry.cpp
    camera.cpp
    frame.cpp
    # --------------no used---------
    # map.cpp
    # mappoint.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)
target_link_libraries( mygeometry
    ${THIRD_PARTY_LIBS}
)

add_executable(main main.cpp)
target_link_libraries( main
    myslam mygeometry
)
