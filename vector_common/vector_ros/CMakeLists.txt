cmake_minimum_required(VERSION 2.8.3)
project(vector_ros)

## Uncomment if the package has a setup.py
catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS roslaunch rospy dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/vector.cfg)

catkin_install_python(PROGRAMS bin/jaco_joint_controller 
                               bin/vector_driver 
                               bin/vector_faultlog_parser 
                               bin/vector_marker_ctrl 
                               bin/vector_move_base 
                               bin/vector_teleop 
                               bin/vector_wd 
                               bin/vector_config_eth
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
                      
