cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_sonar_plugin)


find_package(catkin REQUIRED COMPONENTS roslint
  image_transport 
  roscpp 
  sensor_msgs)
find_package(OpenCV REQUIRED)
find_package(GAZEBO REQUIRED)
find_package(Boost REQUIRED)

set(GAZEBO_ROS_SONAR_PLUGIN "")

catkin_package(
  INCLUDE_DIRS include
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
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(ROSLINT_CPP_OPTS "--extensions=hpp,cpp,c,hh,cc,h")

set(GAZEBO_ROS_SONAR_PLUGIN_SRCS
<<<<<<< HEAD
  src/Sonar.cc
  src/SensorPlugin.cc)
<<<<<<< HEAD
>>>>>>> 3db1d56... Modification of main class to sonar
=======
>>>>>>> 7c8862b... 2
 # src/ImageViewerCaptureTool.cc
 # src/NormalDepthMap.cc
 # src/Tools.cc)
=======
  src/FLSonarRos.cc)
set(GAZEBO_SONAR_PLUGIN_SRCS
  src/FLSonar.cc)
>>>>>>> 2592e5a... Files Organization of Shader implementation

set(GAZEBO_ROS_SONAR_PLUGIN_HEADERS
 include/${PROJECT_NAME}/FLSonar.hh)

roslint_cpp()

roslint_cpp(${GAZEBO_ROS_SONAR_PLUGIN_SRCS}
  ${GAZEBO_ROS_SONAR_PLUGIN_HEADERS})

add_library(FLSonar ${GAZEBO_SONAR_PLUGIN_SRCS})
target_link_libraries(FLSonar ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})
list(APPEND GAZEBO_SONAR_PLUGIN_LIST FLSonar)

add_library(fl_sonar_ros ${GAZEBO_ROS_SONAR_PLUGIN_SRCS})
target_link_libraries(fl_sonar_ros ${catkin_LIBRARIES} ${GAZEBO_SONAR_PLUGIN_LIST})
add_dependencies(fl_sonar_ros ${catkin_EXPORTED_TARGETS})
list(APPEND GAZEBO_ROS_SONAR_PLUGIN_LIST fl_sonar_ros)

install(TARGETS ${GAZEBO_ROS_SONAR_PLUGIN_LIST} ${GAZEBO_SONAR_PLUGIN_LIST}
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
  find_package(OpenCV REQUIRED)

  include_directories(include ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS})

  link_directories(
    ${GAZEBO_LIBRARY_DIRS}
  )

  #Add Media Path
  set(OGRE_GAZEBO_MEIDA_PATH "${PROJECT_SOURCE_DIR}/media")
  configure_file( test/IncludePaths.h.in ${CMAKE_BINARY_DIR}/generated/IncludePaths.h )
  include_directories( ${CMAKE_BINARY_DIR}/generated/ )

  #MESSAGE(FATAL_ERROR ${GAZEBO_LIBRARY_DIRS})
  add_library(gazebo_test_fixture EXCLUDE_FROM_ALL test/ServerFixture.cc)
  target_link_libraries(gazebo_test_fixture ${GAZEBO_LIBRARIES})

  add_rostest_gtest(FLSonar_TEST test/pure_gazebo_test.test test/FLSonar_TEST.cc)
  target_link_libraries(FLSonar_TEST gazebo_test_fixture ${GAZEBO_LIBRARIES} gtest ${GAZEBO_SONAR_PLUGIN_LIST} ${OpenCV_LIBRARIES})

  roslint_add_test()
endif()