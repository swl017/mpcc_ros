cmake_minimum_required(VERSION 3.7)

include_directories("/usr/include/python3.6m/")
include_directories("/usr/include/python2.7/")
include_directories(.)
include_directories(External/blasfeo/lib/include)
include_directories(External/hpipm/lib/include)
include_directories(External/matplotlib)
include_directories(External/Eigen)
include_directories(External/Json/include)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mpcc_ros
 CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs geometry_msgs std_msgs tf
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mpcc_ros.cpp
# )


add_executable(mpcc
        main.cpp
        mpcc_ros.h
        types.cpp
        types.h
        config.h
        Params/params.cpp
        Spline/cubic_spline.cpp
        Spline/arc_length_spline.cpp
        Interfaces/hpipm_interface.cpp
        Interfaces/solver_interface.cpp
        Constraints/constraints.cpp
        Constraints/bounds.cpp
        Cost/cost.cpp
        Model/model.cpp
        Model/integrator.cpp
        MPC/mpc.cpp
        Params/track.cpp
        Params/track.h
        #Plotting/plotting.cpp
        #Plotting/plotting.h
)


find_package(Python COMPONENTS Development)
#find_package(PythonLibs 2.7)
#find_package(PythonLibs REQUIRED)
include_directories(
  ${Python_INCLUDE_DIRS}
)
target_include_directories(mpcc PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(mpcc ${Python_LIBRARIES})

target_link_libraries(mpcc ${CMAKE_CURRENT_SOURCE_DIR}/External/hpipm/lib/lib/libhpipm.a ${CMAKE_CURRENT_SOURCE_DIR}/External/blasfeo/lib/lib/libblasfeo.a m)
target_link_libraries(mpcc ${catkin_LIBRARIES})

install(TARGETS
  mpcc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)