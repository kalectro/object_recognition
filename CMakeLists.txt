cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

find_package(catkin REQUIRED COMPONENTS roscpp rostime pcl_ros sensor_msgs)
find_package(pcl 1.6.1 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rostime pcl_ros sensor_msgs
  DEPENDS system_lib 
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)


add_executable(object_recognition_node src/recognize.cpp)
add_executable(create_pointcloud src/create_pointcloud.cpp)

target_link_libraries(object_recognition_node ${catkin_LIBRARIES})
target_link_libraries(create_pointcloud ${catkin_LIBRARIES})

install(TARGETS object_recognition_node create_pointcloud
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
