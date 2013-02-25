cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

find_package(catkin REQUIRED COMPONENTS roscpp rostime sensor_msgs std_msgs geometry_msgs tf)
find_package(PCL 1.6.1 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rostime sensor_msgs std_msgs geometry_msgs tf
  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(object_recognition_node src/recognize.cpp)
add_executable(create_pointcloud_node src/create_pointcloud.cpp)
add_executable(pcd_descriptor src/pcd_to_descriptors.cpp)

target_link_libraries(object_recognition_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(create_pointcloud_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pcd_descriptor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS object_recognition_node create_pointcloud_node pcd_descriptor
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
