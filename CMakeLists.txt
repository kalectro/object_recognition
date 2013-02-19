cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

find_package(catkin REQUIRED COMPONENTS pcl roscpp rostime)

catkin_package(
  CATKIN_DEPENDS pcl roscpp rostime
  DEPENDS system_lib
)

add_executable(object_recognition_node src/recognize.cpp)

target_link_libraries(object_recognition_node ${catkin_LIBRARIES})

install(TARGETS object_recognition_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
