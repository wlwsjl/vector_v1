cmake_minimum_required(VERSION 2.8.3)
project(vector_config)

find_package(catkin REQUIRED)

catkin_package()

catkin_add_env_hooks(50.vector_config
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(PROGRAMS vector_config.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

