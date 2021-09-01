cmake_minimum_required(VERSION 2.8.3)
project(forward_looking_sonar_gazebo)


find_package(catkin REQUIRED COMPONENTS roslint
  image_transport 
  roscpp 
  sensor_msgs
  sonar_msgs)
find_package(OpenCV REQUIRED)
find_package(GAZEBO REQUIRED)
find_package(Boost REQUIRED)

set(FORWARD_LOOKING_SONAR_GAZEBO "")

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

set(FORWARD_LOOKING_SONAR_GAZEBO_SRCS
  src/FLSonar.cc
  src/FLSonarRos.cc)

set(FORWARD_LOOKING_SONAR_GAZEBO_HEADERS
 include/${PROJECT_NAME}/FLSonar.hh
 include/${PROJECT_NAME}/FLSonarRos.hh
 include/${PROJECT_NAME}/SDFTool.hh)

roslint_cpp()

roslint_cpp(${FORWARD_LOOKING_SONAR_GAZEBO_SRCS}
  ${FORWARD_LOOKING_SONAR_GAZEBO_HEADERS})

add_library(FLSonar src/FLSonar.cc)
target_link_libraries(FLSonar ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES} ${FORWARD_LOOKING_SONAR_GAZEBO_LIST})
list(APPEND FORWARD_LOOKING_SONAR_GAZEBO_LIST FLSonar)

add_library(ForwardLookingSonarGazebo src/FLSonarRos.cc)
target_link_libraries(ForwardLookingSonarGazebo ${catkin_LIBRARIES} ${FORWARD_LOOKING_SONAR_GAZEBO_LIST})
add_dependencies(ForwardLookingSonarGazebo ${catkin_EXPORTED_TARGETS})
list(APPEND FORWARD_LOOKING_SONAR_GAZEBO_LIST ForwardLookingSonarGazebo)

install(TARGETS ${FORWARD_LOOKING_SONAR_GAZEBO_LIST} ${FORWARD_LOOKING_SONAR_GAZEBO_LIST}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY media launch test xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

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

  add_rostest_gtest(FLSonar_TEST test/pure_gazebo_test.test test/FLSonar_TEST.cc)
  target_link_libraries(FLSonar_TEST gazebo_test_fixture ${GAZEBO_LIBRARIES} gtest ${OpenCV_LIBRARIES} ${FORWARD_LOOKING_SONAR_GAZEBO_LIST})
  #Add Media Path
  set_target_properties(FLSonar_TEST PROPERTIES COMPILE_DEFINITIONS "OGRE_MEDIA_PATH=\"${PROJECT_SOURCE_DIR}/media\"")

  roslint_add_test()
endif()
