

add_library(my_basics SHARED
    config.cpp
    eigen_funcs.cpp
    opencv_funcs.cpp
)

add_library(my_display SHARED
    pcl_display.cpp
)

add_library(my_geometry SHARED
    feature_match.cpp
    epipolar_geometry.cpp
)

add_library( my_slam SHARED
    visual_odometry.cpp
    camera.cpp
    frame.cpp
    # --------------not used ---------
    # map.cpp
    # mappoint.cpp
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

target_link_libraries( my_slam
    ${THIRD_PARTY_LIBS} my_geometry my_basics
)

# add_executable(main main.cpp)
# target_link_libraries( main
#     my_slam my_geometry
# )
