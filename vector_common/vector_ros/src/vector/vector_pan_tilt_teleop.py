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
 
 \file   vector_teleop_full_system.py

 \brief  This module contains a class for teleoperating all the vector
         platform DOF with a joystick controller; only works with logitech
         extreme 3d

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from utils import *
from system_defines import *
from vector_msgs.msg import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,Float64
import rospy
import sys
import math
import actionlib


class VectorPanTiltTeleop(object):
    def __init__(self):
        self.is_sim = rospy.get_param('~sim',True)

        """
        Set the mapping for the various commands
        """        
        self.ctrl_map  = dict({'momentary': {'dead_man'     : {'is_button':True,'index':0,'set_val':1},
                                             'man_ovvrd'    : {'is_button':True,'index':1,'set_val':1},
                                             'standby'      : {'is_button':True,'index':2,'set_val':1},
                                             'tractor'      : {'is_button':True,'index':3,'set_val':1},
                                             'estop'        : {'is_button':True,'index':4,'set_val':1},
                                             'pan_tilt_ctl' : {'is_button':True,'index':9,'set_val':1},
                                             'base_ctl'     : {'is_button':True,'index':10,'set_val':1},
                                             'arm_ctl'      : {'is_button':True,'index':11,'set_val':1}},
                               'axis'     : {'left_right'   : {'index' :0, 'invert_axis':False},
                                             'for_aft'      : {'index' :1, 'invert_axis':False},
                                             'twist'        : {'index' :2, 'invert_axis':False},
                                             'flipper'   : {'index' :3, 'invert_axis':False},
                                             'dpad_lr'   : {'index' :4, 'invert_axis':False},
                                             'dpad_ud'   : {'index' :5, 'invert_axis':False}}})
        
        """
        Initialize the debounce logic states
        """
        self.db_cnt = dict()
        self.axis_value = dict()
        self.button_state = dict()
        
        for key, value in self.ctrl_map.iteritems():
            if key == 'momentary':
                for key, value2 in value.iteritems():
                    self.db_cnt[key]=0
                    self.button_state[key]=False
            else:
                self.db_cnt[key]=0
                self.axis_value[key]=0.0            
                 
        self.run_pan_tilt_ctl = False
        
        self.pan_pub = rospy.Publisher('/pan_controller/command', Float64, queue_size=1)
        self.tilt_pub = rospy.Publisher('/tilt_controller/command', Float64, queue_size=1)

        rospy.Subscriber('/joy', Joy, self._vector_teleop)
        
        
    def _parse_joy_input(self,joyMessage):
        
    
        raw_button_states = dict()
        self.button_state = dict()
        
        for key, value in self.ctrl_map.iteritems():
            if key == 'momentary':
                for key2, value2 in value.iteritems():
                    raw_button_states[key2]=True
                    self.button_state[key2]=False
            else:
                for key2, value2 in value.iteritems():  
                    self.axis_value[key2] = 0.0            
         
        
        for key, value in self.ctrl_map.iteritems():
            if key == 'momentary':
                for key2, item in value.iteritems():
                    if item['is_button']:
                        if item['set_val'] == joyMessage.buttons[item['index']]:
                            raw_button_states[key2] &= True
                        else:
                            raw_button_states[key2] = False
                    else:
                        temp = joyMessage.axes[item['index']]
                        if (item['invert_axis']):
                            temp *= -1.0
                        if (temp >= item['set_thresh']):
                            raw_button_states[key2] &= True
                        else:
                            raw_button_states[key2] = False
                     
            
                    if (True == raw_button_states[key2]):
                        self.db_cnt[key2]+=1
                        if (self.db_cnt[key2] > 10):
                            self.db_cnt[key2] = 10
                            self.button_state[key2] = True
                    else:
                        self.button_state[key2] = False
                        self.db_cnt[key2] = 0
            if key == 'axis':
                for key2, item in value.iteritems():
                    temp = joyMessage.axes[item['index']]
                    if (item['invert_axis']):
                        temp *= -1.0
                    self.axis_value[key2] = temp
                    
                    

    def _vector_teleop(self, joyMessage):
        self._parse_joy_input(joyMessage)
        
        if self.button_state['base_ctl']:
            self.run_arm_ctl = False
            self.run_pan_tilt_ctl = False
        elif self.button_state['arm_ctl']:
            self.run_arm_ctl = True
            self.run_pan_tilt_ctl = False
        elif self.button_state['pan_tilt_ctl']:
            self.run_arm_ctl = False
            self.run_pan_tilt_ctl = True
            
        if self.button_state['estop']:
            self.run_arm_ctl = False
            self.run_pan_tilt_ctl = False
            home = Float64()
            home.data = 0.0
            self.pan_pub.publish(home) 
            self.tilt_pub.publish(home) 

      
        if self.run_pan_tilt_ctl:
            if self.button_state['dead_man']:
                pan_cmd = Float64()
                tilt_cmd = Float64()
                pan_cmd.data = self.axis_value['twist'] * 1.05
                tilt_cmd.data = self.axis_value['for_aft'] * 1.4
                self.pan_pub.publish(pan_cmd) 
                self.tilt_pub.publish(tilt_cmd)
        
        


    
