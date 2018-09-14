cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_sonar_plugin)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GAZEBO_ODE REQUIRED gazebo_ode)
set(GAZEBO_ODE_INCLUDE_DIRS "${GAZEBO_ODE_INCLUDE_DIRS}/gazebo")

find_package(catkin REQUIRED COMPONENTS roslint
  )

find_package(GAZEBO REQUIRED)
find_package(Boost REQUIRED)

set(GAZEBO_ROS_SONAR_PLUGIN "")

catkin_package(
  INCLUDE_DIRS include
    ${GAZEBO_ODE_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${GAZEBO_MSG_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
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

#link_directories(
#  ${GAZEBO_LIBRARY_DIRS}
#)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(ROSLINT_CPP_OPTS "--extensions=hpp,cpp,c,hh,cc,h")

set(GAZEBO_ROS_SONAR_PLUGIN_SRCS
  src/Sonar.cc
  src/SensorPlugin.cc)
<<<<<<< HEAD
>>>>>>> 3db1d56... Modification of main class to sonar
=======
>>>>>>> 7c8862b... 2
 # src/ImageViewerCaptureTool.cc
 # src/NormalDepthMap.cc
 # src/Tools.cc)

set(GAZEBO_ROS_SONAR_PLUGIN_HEADERS
 include/${PROJECT_NAME}/SonarPrivate.hh 
 include/${PROJECT_NAME}/Sonar.hh)
 # include/${PROJECT_NAME}/ImageViewerCaptureTool.hh
 # include/${PROJECT_NAME}/NormalDepthMap.hh
 # include/${PROJECT_NAME}/Tools.hh)

roslint_cpp()

roslint_cpp(${GAZEBO_ROS_SONAR_PLUGIN_SRCS}
  ${GAZEBO_ROS_SONAR_PLUGIN_HEADERS})

add_library(SonarVisual ${GAZEBO_ROS_SONAR_PLUGIN_SRCS})
target_link_libraries(SonarVisual ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(SonarVisual ${catkin_EXPORTED_TARGETS})
list(APPEND GAZEBO_ROS_SONAR_PLUGIN_LIST SonarVisual)

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
#install(DIRECTORY urdf
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#  PATTERN "*~" EXCLUDE)


if (CATKIN_ENABLE_TESTING)
  find_package(cmake_modules)
  find_package(rostest REQUIRED)
  find_package(TinyXML REQUIRED)
  find_package(GAZEBO REQUIRED)

  include_directories(include ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS})

  ############# LINEAR FORCE TESTS ###########################
  add_rostest_gtest(sonar_test
    test/sonar_test.test
    test/Sonar_TEST.cc)

  target_link_libraries(sonar_test ${catkin_LIBRARIES}
    SonarPlugin ${GAZEBO_LIBRARIES})
  #message(FATAL_ERROR "${GAZEBO_LIBRARIES}")
  roslint_add_test()
endif()