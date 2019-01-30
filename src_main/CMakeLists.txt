

add_executable(run_vo run_vo.cpp)
target_link_libraries( run_vo
    my_basics
    my_geometry
    my_display
    my_slam
)