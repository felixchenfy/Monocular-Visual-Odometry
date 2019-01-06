
add_executable( test_epipolor_geometry test_epipolor_geometry.cpp )
target_link_libraries( test_epipolor_geometry my_geometry)

# add_executable( test_PnP test_PnP.cpp )
# target_link_libraries( test_PnP my_geometry)

# add_executable( test_config_and_read_yaml test_config_and_read_yaml.cpp )
# target_link_libraries( test_config_and_read_yaml my_slam)

# add_executable(test_eigen_basics test_eigen_basics.cpp)
# target_link_libraries( test_eigen_basics my_basics)

add_executable(test_pcl_viewer test_pcl_viewer.cpp)
target_link_libraries( test_pcl_viewer
    my_basics
    my_display
)

# -------------------------- main --------------------------------
add_executable(run_vo_v1 run_vo_v1.cpp)
target_link_libraries( run_vo_v1
    my_basics
    my_geometry
    my_display
    my_slam
)