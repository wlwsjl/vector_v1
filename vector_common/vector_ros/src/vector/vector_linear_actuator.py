"""--------------------------------------------------------------------
COPYRIGHT 2016 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company") 
for its licensed SI Vector Platform is intended and supplied to you, 
the Company's customer, for use solely and exclusively with Stanley Innovation 
products. The software is owned by the Company and/or its supplier, and is 
protected under applicable copyright laws.  All rights are reserved. Any use in 
violation of the foregoing restrictions may subject the user to criminal 
sanctions under applicable laws, as well as to civil liability for the 
breach of the terms and conditions of this license. The Company may 
immediately terminate this Agreement upon your use of the software with 
any products that are not Stanley Innovation products.

The software was written using Python programming language.  Your use 
of the software is therefore subject to the terms and conditions of the 
OSI- approved open source license viewable at http://www.python.org/.  
You are solely responsible for ensuring your compliance with the Python 
open source license.

You shall indemnify, defend and hold the Company harmless from any claims, 
demands, liabilities or expenses, including reasonable attorneys fees, incurred 
by the Company as a result of any claim or proceeding against the Company 
arising out of or based upon: 

(i) The combination, operation or use of the software by you with any hardware, 
    products, programs or data not supplied or approved in writing by the Company, 
    if such claim or proceeding would have been avoided but for such combination, 
    operation or use.
 
(ii) The modification of the software by or on behalf of you 

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
 \file   vector_linear_actuator.py

 \brief  runs the driver for the linear actuator

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from system_defines import *
from utils import *
from vector_msgs.msg import LinearActuatorCmd
from io_eth import IoEthThread
import multiprocessing
import re
import os
import rospy

class LinearActuator(object):
    def __init__(self):        
        self.init_success = False
        
        """
        Create the thread to run VECTOR Linear actuator command interface
        """
        self.tx_queue_ = multiprocessing.Queue()
        self.rx_queue_ = multiprocessing.Queue()
        self.comm = IoEthThread((os.environ['VECTOR_IP_ADDRESS'],6236),
                                self.tx_queue_,
                                self.rx_queue_,
                                max_packet_size=1248)
                                    
        
        if (False == self.comm.link_up):
            rospy.logerr("Could not open socket for VECTOR linear actuator...exiting")
            self.Shutdown()
            return
        
        """
        Initialize the publishers and subscribers for the node
        """
        self.cmd_data = LinearActuatorCmd()
        rospy.Subscriber("/vector/linear_actuator_cmd", LinearActuatorCmd, self._add_motion_command_to_queue)
        self.init_success = True
    
    def Shutdown(self):
        rospy.loginfo("Shutting down the linear actuator command driver...")
        self.comm.Close()
        self.tx_queue_.close()
        self.rx_queue_.close()
        
    def UpdateVelLimit(self,new_limit):
        cmds = [LINEAR_ACTUATOR_VELOCITY_LIMIT_CMD_ID,[convert_float_to_u32(new_limit)]]
        self._add_command_to_queue(cmds) 
    
    def _add_command_to_queue(self,command):
        
        """
        Create a byte array with the CRC from the command
        """
        cmd_bytes = generate_cmd_bytes(command)
        
        """
        Send it
        """
        self.tx_queue_.put(cmd_bytes)
        
    def _add_motion_command_to_queue(self,command):
        
        """
        Add the command to the queue, platform does command limiting and mapping
        """
        cmds = [LINEAR_ACTUATOR_POSITION_CMD_ID,[convert_float_to_u32(command.desired_position_m)]]
        self._add_command_to_queue(cmds)
