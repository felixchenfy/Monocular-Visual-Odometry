

add_library(basics SHARED
    basics.cpp
    config.cpp
    eigen_funcs.cpp
    opencv_funcs.cpp
    io.cpp
)

add_library(display SHARED
    pcl_display.cpp
    pcl_display_lib.cpp
)

add_library(geometry SHARED
    camera.cpp
    feature_match.cpp
    epipolar_geometry.cpp
    motion_estimation.cpp
)

add_library( optimization SHARED
    g2o_ba.cpp
)


add_library( vo SHARED
    frame.cpp
    vo.cpp
    vo_addFrame.cpp
    map.cpp
    mappoint.cpp
    vo_commons.cpp
)


target_link_libraries( basics
    ${THIRD_PARTY_LIBS} 
)

target_link_libraries( display
    ${THIRD_PARTY_LIBS} basics
)

target_link_libraries( geometry
    ${THIRD_PARTY_LIBS} basics
)

target_link_libraries( optimization
    ${THIRD_PARTY_LIBS} basics
)

target_link_libraries( vo
    ${THIRD_PARTY_LIBS} geometry basics optimization
)


# add_executable(main main.cpp)
# target_link_libraries( main
#     vo geometry
# )
