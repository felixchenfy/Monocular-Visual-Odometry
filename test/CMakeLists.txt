
add_executable( test_epipolor_geometry test_epipolor_geometry.cpp )
target_link_libraries( test_epipolor_geometry geometry)

# add_executable( test_PnP test_PnP.cpp )
# target_link_libraries( test_PnP geometry)

# add_executable( test_config_and_read_yaml test_config_and_read_yaml.cpp )
# target_link_libraries( test_config_and_read_yaml vo)

# add_executable(test_eigen_basics test_eigen_basics.cpp)
# target_link_libraries( test_eigen_basics basics)

# add_executable(test_sophus test_sophus.cpp)
# target_link_libraries( test_sophus ${THIRD_PARTY_LIBS})

# add_executable(test_pcl_display test_pcl_display.cpp)
# target_link_libraries( test_pcl_display
#     basics
#     display
# )

