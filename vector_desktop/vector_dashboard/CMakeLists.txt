cmake_minimum_required(VERSION 2.8.3)
project(vector_dashboard)
find_package(catkin REQUIRED COMPONENTS rospy
                                        rqt_robot_dashboard
                                        vector_msgs)
catkin_python_setup()
catkin_package()
install(PROGRAMS scripts/vector_dashboard
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
