

add_library(my_basics SHARED
    basics.cpp
    config.cpp
    eigen_funcs.cpp
    opencv_funcs.cpp
    io.cpp
)

add_library(my_display SHARED
    pcl_display.cpp
    pcl_display_lib.cpp
)

add_library(my_geometry SHARED
    camera.cpp
    feature_match.cpp
    epipolar_geometry.cpp
    motion_estimation.cpp
)

add_library( my_optimization SHARED
    g2o_ba.cpp
)


add_library( my_slam SHARED
    frame.cpp
    vo.cpp
    vo_addFrame.cpp
    map.cpp
    mappoint.cpp
    commons.cpp
)


target_link_libraries( my_basics
    ${THIRD_PARTY_LIBS} 
)

target_link_libraries( my_display
    ${THIRD_PARTY_LIBS} my_basics
)

target_link_libraries( my_geometry
    ${THIRD_PARTY_LIBS} my_basics
)

target_link_libraries( my_optimization
    ${THIRD_PARTY_LIBS} my_basics
)


target_link_libraries( my_slam
    ${THIRD_PARTY_LIBS} my_geometry my_basics my_optimization
)


# add_executable(main main.cpp)
# target_link_libraries( main
#     my_slam my_geometry
# )
