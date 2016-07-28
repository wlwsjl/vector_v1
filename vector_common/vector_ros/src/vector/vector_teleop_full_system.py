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

Edited 7/25/2016: Vivian Chu, vchu@gatech - included support for simulation

--------------------------------------------------------------------"""
from utils import *
from system_defines import *
from vector_msgs.msg import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,Float64
from trajectory_msgs.msg import JointTrajectoryPoint
from dynamixel_controllers.srv import *  
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
import rospy
import sys
import math
import actionlib


class VectorTeleopFullSystem(object):
    def __init__(self):
        self.is_sim = rospy.get_param('~sim',True)
        self.lincmd = LinearActuatorCmd()
        if (False == self.is_sim):
        
            """
            Subscribe to the configuration message
            """
            self.config_updated = False
            rospy.Subscriber("/vector/feedback/active_configuration", Configuration, self._update_configuration_limits)
            
            start_time = rospy.get_time()            
            while ((rospy.get_time() - start_time) < 10.0) and (False == self.config_updated):
                rospy.sleep(0.05)
            
            if (False == self.config_updated):
                rospy.logerr("Timed out waiting for Vector feedback topics make sure the driver is running")
                sys.exit(0)
                return

            """
            Initialize the linear actuator position if this is the real system
            """
            vector_dynamics = rospy.wait_for_message("/vector/feedback/dynamics", Dynamics)
            self.lincmd.desired_position_m = vector_dynamics.linear_actuator_position_m
            
        else:
            self.x_vel_limit_mps = rospy.get_param('~sim_teleop_x_vel_limit_mps',0.5)
            self.y_vel_limit_mps = rospy.get_param('~sim_teleop_y_vel_limit_mps',0.5)
            self.yaw_rate_limit_rps = rospy.get_param('~sim_teleop_yaw_rate_limit_rps',0.5)
            self.accel_lim = rospy.get_param('~sim_teleop_accel_lim',0.5)
            self.yaw_accel_lim = rospy.get_param('~sim_teleop_yaw_accel_lim',1.0) 

            # Simulation flags for linear actuator 
            self.linact_sub = rospy.Subscriber('/linear_actuator_controller/state', JointTrajectoryControllerState, self._update_simulation_linear_actuator, queue_size=1)
            self.sim_lin_actuator_position = 0.0 # init to 0 for now
            self.sim_lin_init = False
            self.last_arm_update = rospy.get_time()
    
        """
        Set the mapping for the various commands
        """        
        self.ctrl_map  = dict({'momentary': {'dead_man'     : {'is_button':True,'index':0,'set_val':1},
                                             'man_ovvrd'    : {'is_button':True,'index':1,'set_val':1},
                                             'standby'      : {'is_button':True,'index':2,'set_val':1},
                                             'tractor'      : {'is_button':True,'index':3,'set_val':1},
                                             'estop'        : {'is_button':True,'index':4,'set_val':1},
                                             'pan_tilt_ctl' : {'is_button':True,'index':8,'set_val':1},
                                             'base_ctl'     : {'is_button':True,'index':9,'set_val':1},
                                             'arm_ctl_right': {'is_button':True,'index':10,'set_val':1},
                                             'arm_ctl_left' : {'is_button':True,'index':11,'set_val':1}},
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
                 

        self.send_cmd_none = False
        self.no_motion_commands = True
        self.last_motion_command_time = 0.0
        self.last_joy = rospy.get_time()
        self._last_gripper_val = 0.0
        self.run_arm_ctl_right = False
        self.run_arm_ctl_left = False
        self.run_pan_tilt_ctl = False
        self._init_pan_tilt = True
        self._last_angles = [0.0,0.0]
            
        self.cfg_cmd = ConfigCmd()
        self.cfg_pub = rospy.Publisher('/vector/gp_command', ConfigCmd, queue_size=10)
        
        self.motion_cmd = Twist()
        self.limited_cmd = Twist()
        self.motion_pub = rospy.Publisher('/vector/teleop/cmd_vel', Twist, queue_size=10)
        self.override_pub = rospy.Publisher("/vector/manual_override/cmd_vel",Twist, queue_size=10)
        self.linpub = rospy.Publisher("/vector/linear_actuator_cmd",LinearActuatorCmd,queue_size=1)
        
        self.arm_pub = [0]*2
        self.gripper_pub = [0]*2
        self.arm_pub[0] = rospy.Publisher('/vector/right_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)
        self.gripper_pub[0] = rospy.Publisher('/vector/right_gripper/cmd', GripperCmd, queue_size=10)

        self.arm_pub[1] = rospy.Publisher('/vector/left_arm/cartesian_vel_cmd', JacoCartesianVelocityCmd, queue_size=10)
        self.gripper_pub[1] = rospy.Publisher('/vector/left_gripper/cmd', GripperCmd, queue_size=10)
 
        self.pan_pub = rospy.Publisher('/pan_controller/command', Float64, queue_size=1)
        self.tilt_pub = rospy.Publisher('/tilt_controller/command', Float64, queue_size=1)

        rospy.Subscriber('/joy', Joy, self._vector_teleop)
       
    def _update_simulation_linear_actuator(self, msg):
        self.sim_lin_actuator_position = msg.actual.positions[0] 
 
    def _update_configuration_limits(self,config):
        
        self.x_vel_limit_mps = config.teleop_x_vel_limit_mps
        self.y_vel_limit_mps = config.teleop_y_vel_limit_mps
        self.yaw_rate_limit_rps = config.teleop_yaw_rate_limit_rps
        self.accel_lim = config.teleop_accel_limit_mps2
        self.yaw_accel_lim = config.teleop_yaw_accel_limit_rps2
        self.config_updated = True
        
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
            self.run_arm_ctl_right = False
            self.run_arm_ctl_left = False
            self.run_pan_tilt_ctl = False
            self._init_pan_tilt = False
        elif self.button_state['arm_ctl_right']:
            self.run_arm_ctl_right = True
            self.run_arm_ctl_left = False
            self.run_pan_tilt_ctl = False
            self._init_pan_tilt = False
        elif self.button_state['arm_ctl_left']:
            self.run_arm_ctl_right = False
            self.run_arm_ctl_left = True
            self.run_pan_tilt_ctl = False
            self._init_pan_tilt = False
        elif self.button_state['pan_tilt_ctl']:
            self.run_arm_ctl = False
            self.run_arm_ctl_right = False
            self.run_arm_ctl_left = False
            self.run_pan_tilt_ctl = True
            self._init_pan_tilt = True
            
        if self.button_state['estop']:
            self.run_arm_ctl = False
            self.run_pan_tilt_ctl = False
            self._init_pan_tilt = False
            arm_cmd = JacoCartesianVelocityCmd()
            arm_cmd.header.stamp=rospy.get_rostime()
            arm_cmd.header.frame_id=''
            self.arm_pub[0].publish(arm_cmd)
            self.arm_pub[1].publish(arm_cmd)
            home = Float64()
            home.data = 0.0
            self.pan_pub.publish(home) 
            self.tilt_pub.publish(home) 
   
        
        if self.run_arm_ctl_right or self.run_arm_ctl_left:
            arm_cmd = JacoCartesianVelocityCmd()
            arm_cmd.header.stamp=rospy.get_rostime()
            arm_cmd.header.frame_id=''
            gripper_cmd = GripperCmd()
            
            if self.run_arm_ctl_right:
                arm_idx = 0
            else:
                arm_idx = 1
            
            if self.button_state['dead_man']:
                arm_cmd.x = self.axis_value['left_right'] * 0.1
                arm_cmd.z = self.axis_value['for_aft'] * 0.1
                if not self.button_state['man_ovvrd']:
                    arm_cmd.y = self.axis_value['twist'] * 0.1
                else:
                    # Check if we're in simulation - if so set the last known position
                    if self.is_sim == True:
                        if self.sim_lin_init == False:
                            self.lincmd.desired_position_m = self.sim_lin_actuator_position
                            self.sim_lin_init = True
                        
                    dt = rospy.get_time() - self.last_arm_update
                    self.lincmd.desired_position_m += (self.axis_value['twist'] * 0.05) * dt
                    
                    if (self.lincmd.desired_position_m  > 0.855):
                        self.lincmd.desired_position_m  = 0.855
                    elif self.lincmd.desired_position_m  < 0.0:
                        self.lincmd.desired_position_m  = 0.0
                    
                    self.lincmd.header.stamp = rospy.get_rostime()
                    self.lincmd.header.frame_id=''
                    self.linpub.publish(self.lincmd)
                    self.lincmd.header.seq+=1
                self.last_arm_update = rospy.get_time()
                arm_cmd.theta_y = self.axis_value['dpad_ud'] * 100.0
                arm_cmd.theta_x = self.axis_value['dpad_lr'] * 100.0

                if self.button_state['standby']:
                    arm_cmd.theta_z = 100.0                
                elif self.button_state['tractor']:
                    arm_cmd.theta_z = -100.0                 
            
            gripper_val =  (self.axis_value['flipper'] + 1.0)/2.0
            
            if abs(self._last_gripper_val-gripper_val) > 0.05: 
                gripper_cmd.position = gripper_val * 0.085
                gripper_cmd.speed = 0.05
                gripper_cmd.force = 100.0
            
                self.gripper_pub[arm_idx].publish(gripper_cmd)
                self._last_gripper_val = gripper_val
    
            self.arm_pub[arm_idx].publish(arm_cmd)
        elif self.run_pan_tilt_ctl:
            if self._init_pan_tilt:
               
                # Check if we're in sim - if so use default speed
                if self.is_sim == False:
                    rospy.wait_for_service('/pan_controller/set_speed')
                    rospy.wait_for_service('/tilt_controller/set_speed')
 
                    try:
                        set_speed = rospy.ServiceProxy('/pan_controller/set_speed', SetSpeed)
                        resp1 = set_speed(1.0)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e

                    try:
                        set_speed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed)
                        resp1 = set_speed(1.0)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e
                self._init_pan_tilt = False
            
            if self.button_state['dead_man']:
                pan = self.axis_value['twist'] * 1.05
                tilt = self.axis_value['for_aft'] * 1.4
                pan_cmd = Float64()
                tilt_cmd = Float64()
                pan_cmd.data = pan
                tilt_cmd.data = tilt
                
                if abs(self._last_angles[0] - pan) > 0.05:
                    self.pan_pub.publish(pan_cmd)
                    self._last_angles[0] = pan  
                if abs(self._last_angles[1] - tilt) > 0.05:
                    self.tilt_pub.publish(tilt_cmd)
                    self._last_angles[1] = tilt
        else:
            if self.button_state['estop']:
                self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
                self.cfg_cmd.gp_param = DTZ_REQUEST  
            elif self.button_state['standby']:
                self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
                self.cfg_cmd.gp_param = STANDBY_REQUEST
            elif self.button_state['tractor']:
                self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
                self.cfg_cmd.gp_param = TRACTOR_REQUEST
            else:
                self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
                self.cfg_cmd.gp_param = 0
                
            if ('GENERAL_PURPOSE_CMD_NONE' != self.cfg_cmd.gp_cmd):
                self.cfg_cmd.header.stamp = rospy.get_rostime()
                self.cfg_pub.publish(self.cfg_cmd)
                self.cfg_cmd.header.seq
                self.send_cmd_none = True
            elif (True == self.send_cmd_none):
                self.cfg_cmd.header.stamp = rospy.get_rostime()
                self.cfg_pub.publish(self.cfg_cmd)
                self.cfg_cmd.header.seq
                self.send_cmd_none = False
            elif (False == self.send_cmd_none):
                if self.button_state['dead_man']:
                    self.motion_cmd.linear.x =  (self.axis_value['for_aft'] * self.x_vel_limit_mps)
                    self.motion_cmd.linear.y =  (self.axis_value['left_right'] * self.y_vel_limit_mps)
                    self.motion_cmd.angular.z = (self.axis_value['twist'] * self.yaw_rate_limit_rps)
                    self.last_motion_command_time = rospy.get_time()
                  
                else:
                    self.motion_cmd.linear.x = 0.0
                    self.motion_cmd.linear.y = 0.0
                    self.motion_cmd.angular.z = 0.0

                dt = rospy.get_time() - self.last_joy
                self.last_joy = rospy.get_time()
                
                if (dt >= 0.01):

                    self.limited_cmd.linear.x = slew_limit(self.motion_cmd.linear.x,
                                                           self.limited_cmd.linear.x,
                                                           self.accel_lim, dt)
                    self.limited_cmd.linear.y = slew_limit(self.motion_cmd.linear.y,
                                                           self.limited_cmd.linear.y,
                                                           self.accel_lim, dt)
                    self.limited_cmd.angular.z = slew_limit(self.motion_cmd.angular.z,
                                                           self.limited_cmd.angular.z,
                                                           self.yaw_accel_lim, dt)

                    if ((rospy.get_time() - self.last_motion_command_time) < 2.0):
         
                        
                        self.motion_pub.publish(self.limited_cmd)
                        
                        if self.button_state['man_ovvrd'] and self.button_state['man_ovvrd']:
                            self.override_pub.publish(self.motion_cmd)


