cmake_minimum_required(VERSION 3.1.0)
project(pitag_ros)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

set (CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(TinyXML REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  image_transport
  sensor_msgs
  cv_bridge
  tf2
  tf2_geometry_msgs
)

catkin_package(
  LIBRARIES pitag_ros_nodelet
  CATKIN_DEPENDS roscpp nodelet
)

###########
## Build ##
###########
include_directories(
  common/include
  ${OpenCV_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(project_CPP_FILES
  AbstractFiducialModel.cpp
  FiducialModelPi.cpp
  ellipserefine.cpp
)

add_library(${PROJECT_NAME} 
  ${project_CPP_FILES}
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${TinyXML_LIBRARIES}
)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-D__LINUX__")

# add_executable(pitag-test
# pitag-test.cpp
#)
#target_link_libraries(pitag-test
# ${PROJECT_NAME}
# ${OpenCV_LIBS}
# ${TinyXML_LIBRARIES}
# )
# set_target_properties(pitag-test PROPERTIES COMPILE_FLAGS "-D__LINUX__")

add_library(pitag_ros_nodelet
  pitag_ros_nodelet.cpp
)

set_target_properties(pitag_ros_nodelet PROPERTIES COMPILE_FLAGS "-D__LINUX__")
target_link_libraries(pitag_ros_nodelet
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${TinyXML_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(pitag_ros_node
  pitag_ros.cpp
)

set_target_properties(pitag_ros_node PROPERTIES COMPILE_FLAGS "-D__LINUX__")
target_link_libraries(pitag_ros_node
  pitag_ros_nodelet
  ${catkin_LIBRARIES}
)
