cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_sonar_plugin)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GAZEBO_ODE REQUIRED gazebo_ode)
set(GAZEBO_ODE_INCLUDE_DIRS "${GAZEBO_ODE_INCLUDE_DIRS}/gazebo")

find_package(catkin REQUIRED COMPONENTS roslint
  )

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)

set(GAZEBO_ROS_SONAR_PLUGIN "")

catkin_package(
  INCLUDE_DIRS include
    ${GAZEBO_ODE_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${GAZEBO_MSG_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS geometry_msgs
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/test/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_ODE_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(ROSLINT_CPP_OPTS "--extensions=hpp,cpp,c,hh,cc,h")

set(GAZEBO_ROS_SONAR_PLUGIN_SRCS
  src/)

set(GAZEBO_ROS_SONAR_PLUGIN_HEADERS
  include/${PROJECT_NAME}/)

roslint_cpp()

roslint_cpp(${GAZEBO_ROS_SONAR_PLUGIN_SRCS}
  ${GAZEBO_ROS_SONAR_PLUGIN_HEADERS})

add_library(SonarPlugin ${GAZEBO_ROS_SONAR_PLUGIN_SRCS})
target_link_libraries(SonarPlugin ${catkin_LIBRARIES})
add_dependencies(SonarPlugin ${catkin_EXPORTED_TARGETS})
list(APPEND GAZEBO_ROS_SONAR_PLUGIN_LIST SonarPlugin)

install(TARGETS ${GAZEBO_ROS_SONAR_PLUGIN_LIST}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

# Install urdf files
install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE)
