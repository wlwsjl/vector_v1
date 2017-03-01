cmake_minimum_required(VERSION 2.8.3)
project(vector_network)

find_package(catkin REQUIRED)

catkin_package()

install(FILES vector_network_config.bash
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_env_hooks(50.vector_network_config
  SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)


