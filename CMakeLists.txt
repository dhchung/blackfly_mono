cmake_minimum_required(VERSION 3.0.2)
project(blackfly_mono)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)


find_package(OpenCV REQUIRED)

FIND_LIBRARY(SPINNAKER_LIBRARIES Spinnaker /opt/spinnaker/lib/)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES blackfly_mono
 CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

file(GLOB CLASS_SOURCES
  "classes/*.cpp"
)

include_directories(
  include
	/opt/spinnaker/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp
  ${CLASS_SOURCES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${SPINNAKER_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)

add_executable(still_image_node src/still_image.cpp
  ${CLASS_SOURCES}
)

target_link_libraries(still_image_node
  ${catkin_LIBRARIES}
  ${SPINNAKER_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)