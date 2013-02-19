cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

find_package(catkin REQUIRED COMPONENTS roscpp rostime pcl_ros)
find_package(pcl 1.6.1 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rostime pcl_ros
  DEPENDS system_lib 
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

#include_directories(${PCL_INCLUDE_DIRS})

add_executable(object_recognition_node src/recognize.cpp)

target_link_libraries(object_recognition_node ${catkin_LIBRARIES})

install(TARGETS object_recognition_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
