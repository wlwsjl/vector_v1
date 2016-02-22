cmake_minimum_required(VERSION 2.8.3)
project(vector_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(DIRECTORY msg FILES 
                                ConfigCmd.msg 
                                Battery.msg 
                                Status.msg 
                                Propulsion.msg 
                                Dynamics.msg 
                                Configuration.msg 
                                Faultlog.msg
                                JacoJointCmd.msg
                                JacoStatus.msg
                                GripperCmd.msg
                                GripperStat.msg
                                LinearActuatorCmd.msg
                                JacoCartesianVelocityCmd.msg)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

                      
