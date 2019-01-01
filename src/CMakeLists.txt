add_library( mylibs SHARED
    config.cpp
    visual_odometry.cpp
    camera.cpp
    frame.cpp
    feature_match.cpp
    epipolar_geometry.cpp

    # --------------no used---------
    # map.cpp
    # mappoint.cpp
)

target_link_libraries( mylibs
    ${THIRD_PARTY_LIBS}
)

add_executable(main main.cpp)
target_link_libraries( main
    mylibs
)
