cmake_minimum_required(VERSION 2.8.3)
project(vector_costmap_plugins)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  roscpp
  std_msgs
  tf
  jsk_recognition_msgs
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vector_costmap
  CATKIN_DEPENDS
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    jsk_recognition_msgs
    roscpp
    tf
  DEPENDS
    Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(vector_costmap plugins/polygon_obstacle_layer.cpp)
target_link_libraries(vector_costmap ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(vector_costmap ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

install(TARGETS vector_costmap
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)

install(FILES costmap_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
