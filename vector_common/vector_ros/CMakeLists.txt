cmake_minimum_required(VERSION 2.8.3)
project(vector_ros)

## Uncomment if the package has a setup.py
catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS roslaunch rospy dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/vector.cfg)

file(GLOB SCRIPTS bin/*)
catkin_install_python(PROGRAMS ${SCRIPTS} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                      
