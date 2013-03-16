cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

find_package(catkin REQUIRED COMPONENTS roscpp rostime sensor_msgs std_msgs geometry_msgs tf message_generation image_transport)
find_package(PCL 1.6.1 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  Shot352.msg
  Shot352_bundle.msg
	Shot1344.msg
	Shot1344_bundle.msg
	NARF.msg
	NARF_bundle.msg
)
generate_messages(DEPENDENCIES std_msgs object_recognition)

catkin_package(
  CATKIN_DEPENDS roscpp rostime sensor_msgs std_msgs geometry_msgs tf image_transport
  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(object_recognition_node src/recognize.cpp)
add_executable(pointcloud_descriptor src/pointcloud_to_descriptors.cpp)
add_executable(filter_pointcloud src/segmentation.cpp)
add_executable(NARF_descriptor src/depthimage_to_NARF.cpp)


target_link_libraries(object_recognition_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(pointcloud_descriptor ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(filter_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(NARF_descriptor ${catkin_LIBRARIES} ${PCL_LIBRARIES})


install(TARGETS object_recognition_node pointcloud_descriptor filter_pointcloud NARF_descriptor
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY pcds
				DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
