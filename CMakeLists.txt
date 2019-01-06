cmake_minimum_required( VERSION 2.8 )
project ( my_slam )


# compile settings
set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

# set ccache (for speeding up)
find_program(CCACHE_FOUND ccache) 
if(CCACHE_FOUND) 
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache) 
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) 
endif(CCACHE_FOUND) 
set (CMAKE_CXX_FLAGS "-DPCL_ONLY_CORE_POINT_TYPES=ON -DNO_EXPLICIT_INSTANTIATIONS") 

############### dependencies ######################

# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV 4.0 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus 
# find_package( Sophus REQUIRED )
# include_directories( ${Sophus_INCLUDE_DIRS} )
# G2O
# find_package( G2O REQUIRED )
# include_directories( ${G2O_INCLUDE_DIRS} )
# PCL 
find_package( PCL REQUIRED ) 
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# Combine the above libraries
set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    # libSophus.so
    # g2o_core g2o_stuff g2o_types_sba
)

############### dependencies ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )