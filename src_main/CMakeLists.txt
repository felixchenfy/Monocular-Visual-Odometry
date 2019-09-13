

add_executable(run_vo run_vo.cpp)
target_link_libraries( run_vo
    basics
    geometry
    display
    vo
)