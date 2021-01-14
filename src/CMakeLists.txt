cmake_minimum_required(VERSION 3.7)

include_directories("/usr/include/python3.6m/")
include_directories("/usr/include/python2.7/")
include_directories(.)
include_directories(
mpcc_controller/
mpcc_controller/External/blasfeo/lib/include
mpcc_controller/External/hpipm/lib/include
mpcc_controller/External/matplotlib
mpcc_controller/External/Eigen
mpcc_controller/External/Json/include
mpcc_controller/Params
mpcc_controller/Spline
mpcc_controller/Interfaces
mpcc_controller/Constraints
mpcc_controller/Cost
mpcc_controller/Model
mpcc_controller/MPC
)


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


add_executable(mpcc_controller_node
  mpcc_controller_node.cpp
  mpcc_controller/mpcc_ros.h
  mpcc_controller/types.cpp
  mpcc_controller/types.h
  mpcc_controller/config.h
  mpcc_controller/Params/params.cpp
  mpcc_controller/Spline/cubic_spline.cpp
  mpcc_controller/Spline/arc_length_spline.cpp
  mpcc_controller/Interfaces/hpipm_interface.cpp
  mpcc_controller/Interfaces/solver_interface.cpp
  mpcc_controller/Constraints/constraints.cpp
  mpcc_controller/Constraints/bounds.cpp
  mpcc_controller/Cost/cost.cpp
  mpcc_controller/Model/model.cpp
  mpcc_controller/Model/integrator.cpp
  # mpcc_controller/# Tests/spline_test.cpp
  # mpcc_controller/# Tests/cost_test.cpp
  # mpcc_controller/# Tests/model_integrator_test.cpp
  # mpcc_controller/# Tests/constratins_test.cpp
  mpcc_controller/MPC/mpc.cpp
  mpcc_controller/Params/track.cpp
  mpcc_controller/Params/track.h
  # mpcc_controller/#Plotting/plotting.cpp
  # mpcc_controller/#Plotting/plotting.h
)


find_package(Python COMPONENTS Development)
#find_package(PythonLibs 2.7)
#find_package(PythonLibs REQUIRED)
include_directories(
  ${Python_INCLUDE_DIRS}
)
target_include_directories(mpcc_controller_node PRIVATE ${Python_INCLUDE_DIRS})
target_link_libraries(mpcc_controller_node ${Python_LIBRARIES})

target_link_libraries(mpcc_controller_node
${CMAKE_CURRENT_SOURCE_DIR}/mpcc_controller/External/hpipm/lib/lib/libhpipm.a
${CMAKE_CURRENT_SOURCE_DIR}/mpcc_controller/External/blasfeo/lib/lib/libblasfeo.a m
${catkin_LIBRARIES}
)

install(TARGETS
  mpcc_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)