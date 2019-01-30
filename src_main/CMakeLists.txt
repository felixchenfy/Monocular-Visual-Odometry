

add_executable(run_vo_v1 run_vo_v1.cpp)
target_link_libraries( run_vo_v1
    my_basics
    my_geometry
    my_display
    my_slam
)