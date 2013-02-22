cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

find_package(catkin REQUIRED COMPONENTS roscpp rostime sensor_msgs std_msgs geometry_msgs)
find_package(PCL)

catkin_package(
  CATKIN_DEPENDS roscpp rostime sensor_msgs std_msgs geometry_msgs
  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(object_recognition_node src/recognize.cpp)

target_link_libraries(object_recognition_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS object_recognition_node 
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
