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
 
 \file   vector_comm.py

 \brief  runs the driver

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from system_defines import *
from utils import *
from vector_msgs.msg import *
from geometry_msgs.msg import Twist
from vector_ros.cfg import vectorConfig
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.msg import Config
from io_eth import IoEthThread
from vector_data_classes import VECTOR_DATA
from vector_linear_actuator import LinearActuator
import multiprocessing
import rospy
import select
import threading
import re
import os
"""
Dictionary for all VECTOR configuration command ID's
"""
command_ids = dict({"GENERAL_PURPOSE_CMD_NONE":                   0,
                    "GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE":   1,
                    "GENERAL_PURPOSE_CMD_SEND_FAULTLOG":          2,
                    "GENERAL_PURPOSE_CMD_RESET_ODOMETRY":         3,
                    "GENERAL_PURPOSE_CMD_RESET_PARAMS_TO_DEFAULT":4,
                    "SIC_CMD_RESET_IN_DIAGNOSTIC_MODE":1001,
                    "VECTOR_CMD_UNLOCK_PID_GAINS": 2001})

class VectorDriver:
    def __init__(self):

        """
        Variables to track communication frequency for debugging
        """
        self.summer=0
        self.samp = 0
        self.avg_freq = 0.0
        self.start_frequency_samp = False
        self.need_to_terminate = False
        self.flush_rcvd_data=True
        self.update_base_local_planner = False
        self.parameter_server_is_updating = False
        self.last_move_base_update = rospy.Time.now().to_sec()

        """
        Initialize the publishers for VECTOR
        """
        self.vector_data = VECTOR_DATA()
        
        """
        Start the thread for the linear actuator commands
        """
        self._linear = LinearActuator()
        if (False == self._linear.init_success):
            rospy.logerr("Could not initialize the linear actuator interface! exiting...")
            return    
        
        """
        Initialize faultlog related items
        """
        self.is_init = True
        self.extracting_faultlog = False
        
        """
        Initialize the dynamic reconfigure server for VECTOR
        """
        self.param_server_initialized = False
        self.dyn_reconfigure_srv = Server(vectorConfig, self._dyn_reconfig_callback)

        """
        Wait for the parameter server to set the configs and then set the IP address from that.
        Note that this must be the current ethernet settings of the platform. If you want to change it
        set the ethernet settings at launch to the current ethernet settings, power up, change them, power down, set the
        the ethernet settings at launch to the new ones and relaunch
        """
        r = rospy.Rate(10)
        start_time = rospy.Time.now().to_sec()
        while ((rospy.Time.now().to_sec() - start_time) < 3.0) and (False == self.param_server_initialized):
            r.sleep()
        
        if (False == self.param_server_initialized):
            rospy.logerr("Parameter server not found, you must pass an initial yaml in the launch! exiting...")
            return            
        
        """
        Create the thread to run VECTOR communication
        """
        self.tx_queue_ = multiprocessing.Queue()
        self.rx_queue_ = multiprocessing.Queue()
        self.comm = IoEthThread((os.environ['VECTOR_IP_ADDRESS'],int(os.environ['VECTOR_IP_PORT_NUM'])),
                                self.tx_queue_,
                                self.rx_queue_,
                                max_packet_size=1248)
                                    
        
        if (False == self.comm.link_up):
            rospy.logerr("Could not open socket for VECTOR...")
            self.comm.close()
            return
        
        """
        Initialize the publishers and subscribers for the node
        """
        self.faultlog_pub = rospy.Publisher('/vector/feedback/faultlog', Faultlog, queue_size=10,latch=True)
        rospy.Subscriber("/vector/cmd_vel", Twist, self._add_motion_command_to_queue)
        rospy.Subscriber("/vector/gp_command",ConfigCmd,self._add_config_command_to_queue)
        rospy.Subscriber("/move_base/DWAPlannerROS/parameter_updates",Config,self._update_move_base_params)

        """
        Start the receive handler thread
        """
        self.terminate_mutex = threading.RLock()
        self.last_rsp_rcvd = rospy.Time.now().to_sec()
        self._rcv_thread   = threading.Thread(target = self._run)
        self._rcv_thread.start()
        
        """
        Start streaming continuous data
        """
        rospy.loginfo("Stopping the data stream")
        if (False == self._continuous_data(False)):
            rospy.logerr("Could not stop VECTOR communication stream")
            self.Shutdown()
            return
        
        """
        Extract the faultlog at startup
        """
        self.flush_rcvd_data=False
        rospy.loginfo("Extracting the faultlog")
        self.extracting_faultlog = True
        
        if (False == self._extract_faultlog()):
            rospy.logerr("Could not get retrieve VECTOR faultlog")
            self.Shutdown()
            return          
        
        """
        Start streaming continuous data
        """
        rospy.loginfo("Starting the data stream")
        if (False == self._continuous_data(True)):
            rospy.logerr("Could not start VECTOR communication stream")
            self.Shutdown()
            return
            
        self.start_frequency_samp = True
        
        """
        Force the configuration to update the first time to ensure that the variables are set to
        the correct values on the machine
        """
        if (False == self._force_machine_configuration(True)):
            rospy.logerr("Initial configuration parameteters my not be valid, please check them in the yaml file")
            rospy.logerr("The ethernet address must be set to the present address at startup, to change it:")
            rospy.logerr("start the machine; change the address using rqt_reconfigure; shutdown; update the yaml and restart")
            self.Shutdown()
            return
            
            
        """
        Uncomment the line below to always unlock gain tuning and force the upload based on the parameters in the yaml file
        you must have the key to do this because it can be dangerous if one sets unstable gains. 0x00000000 needs to be replaced
        by the key
        """
        #self._unlock_gain_tuning(0x00000000)
        #self._force_pid_configuration(True)
        
        """
        Finally update the values for dynamic reconfigure with the ones reported by the machine
        """
        new_config = dict({"x_vel_limit_mps" : self.vector_data.config_param.machcfg.x_vel_limit_mps,
                           "y_vel_limit_mps" : self.vector_data.config_param.machcfg.y_vel_limit_mps,
                           "accel_limit_mps2" : self.vector_data.config_param.machcfg.accel_limit_mps2,
                           "dtz_decel_limit_mps2": self.vector_data.config_param.machcfg.dtz_decel_limit_mps2,
                           "yaw_rate_limit_rps" : self.vector_data.config_param.machcfg.yaw_rate_limit_rps,
                           "wheel_diameter_m": self.vector_data.config_param.machcfg.wheel_diameter_m,
                           "wheel_base_length_m": self.vector_data.config_param.machcfg.wheelbase_length_m,
                           "wheel_track_width_m": self.vector_data.config_param.machcfg.wheel_track_width_m,
                           "gear_ratio" : self.vector_data.config_param.machcfg.gear_ratio,
                           "motion_while_charging" : ((self.vector_data.config_param.machcfg.config_bitmap >> DISABLE_AC_PRESENT_CSI_SHIFT) & 1) ^ 1,
                           "motion_ctl_input_filter" : (self.vector_data.config_param.machcfg.config_bitmap >> MOTION_MAPPING_FILTER_SHIFT) & VALID_MOTION_MAPPING_FILTER_MASK,
                           
                           "p_gain_rps_per_rps" : self.vector_data.config_param.ctlconfig.p_gain_rps_per_rps,
                           "i_gain_rps_per_rad" : self.vector_data.config_param.ctlconfig.i_gain_rps_per_rad,
                           "d_gain_rps_per_rps2" : self.vector_data.config_param.ctlconfig.d_gain_rps_per_rps2,
                           "fdfwd_gain_rps_per_motor_rps" : self.vector_data.config_param.ctlconfig.fdfwd_gain_rps_per_motor_rps,
                           "p_error_limit_rps" : self.vector_data.config_param.ctlconfig.p_error_limit_rps,
                           "i_error_limit_rad" : self.vector_data.config_param.ctlconfig.i_error_limit_rad,
                           "i_error_drain_rate_rad_per_frame" : self.vector_data.config_param.ctlconfig.i_error_drain_rate_rad_per_frame,
                           "input_target_limit_rps" : self.vector_data.config_param.ctlconfig.input_target_limit_rps,
                           "output_limit_rps" : self.vector_data.config_param.ctlconfig.input_target_limit_rps})

        self.dyn_reconfigure_srv.update_configuration(new_config)
        
        rospy.loginfo("Vector Driver is up and running")
    
    def Shutdown(self):
        with self.terminate_mutex:
            self.need_to_terminate = True
        rospy.loginfo("Vector Driver has called the Shutdown method, terminating")
        self._linear.Shutdown()
        self.comm.Close()
        self.tx_queue_.close()
        self.rx_queue_.close()    
    
    def _run(self):
        
        while not self.need_to_terminate:
            """
            Run until signaled to stop
            Perform the actions defined based on the flags passed out
            """
            result = select.select([self.rx_queue_._reader],[],[],0.02)
            if len(result[0]) > 0:
                data = result[0][0].recv()
                with self.terminate_mutex:
                    if not self.need_to_terminate:
                        self._handle_rsp(data)

        
    def _add_command_to_queue(self,command):
        
        """
        Create a byte array with the CRC from the command
        """
        cmd_bytes = generate_cmd_bytes(command)
        
        """
        Send it
        """
        self.tx_queue_.put(cmd_bytes)
        
    def _update_rcv_frq(self):
        if (True == self.start_frequency_samp):
            self.samp+=1
            self.summer+=1.0/(rospy.Time.now().to_sec() - self.last_rsp_rcvd)
            self.avg_freq = self.summer/self.samp
        self.last_rsp_rcvd = rospy.Time.now().to_sec()
                        
    def _handle_rsp(self,data_bytes):
        
        self._update_rcv_frq()
        if (True == self.flush_rcvd_data):
            return
        

        valid_data,rsp_data = validate_response(data_bytes)
        if (False == valid_data):
            rospy.logerr("bad vector data packet")
            return
        if (self.extracting_faultlog):
            if (len(rsp_data) == NUMBER_OF_FAULTLOG_WORDS):
                self.extracting_faultlog = False
                faultlog_msg = Faultlog()
                faultlog_msg.data = rsp_data
                self.faultlog_pub.publish(faultlog_msg)
        elif (len(rsp_data) == NUMBER_OF_VECTOR_RSP_WORDS):
            
            header_stamp = self.vector_data.status.parse(rsp_data[START_STATUS_BLOCK:END_STATUS_BLOCK])
            wheel_circum = self.vector_data.config_param.parse(rsp_data[START_APP_CONFIG_BLOCK:END_FRAM_CONFIG_BLOCK],header_stamp)
            self.vector_data.auxiliary_power.parse(rsp_data[START_BATTERY_DATA_BLOCK:END_BATTERY_DATA_BLOCK],header_stamp)
            self.vector_data.propulsion.parse(rsp_data[START_PROPULSION_DATA_BLOCK:END_PROPULSION_DATA_BLOCK],header_stamp)
            self.vector_data.dynamics.parse(rsp_data[START_DYNAMICS_DATA_BLOCK:END_DYNAMICS_DATA_BLOCK],header_stamp,wheel_circum)            
            self.vector_data.imu.parse_data(rsp_data[START_IMU_DATA_BLOCK:END_IMU_DATA_BLOCK],header_stamp)
            
            rospy.logdebug("feedback received from vector")
        
    def _add_motion_command_to_queue(self,command):
        
        """
        Add the command to the queue, platform does command limiting and mapping
        """
        cmds = [MOTION_CMD_ID,[convert_float_to_u32(command.linear.x),
                               convert_float_to_u32(command.linear.y),
                               convert_float_to_u32(command.angular.z)]]
        if (False == self.parameter_server_is_updating):
            self._add_command_to_queue(cmds)
            
    def _add_config_command_to_queue(self,command):
        try:
            cmds = [GENERAL_PURPOSE_CMD_ID,[command_ids[command.gp_cmd],command.gp_param]]
            self._add_command_to_queue(cmds)
        except:
            rospy.logerr("Config param failed, it is probably not known")
            return

    def _dyn_reconfig_callback(self,config,level):
        """
        Note: for some reason the commands collide (queue gets garbled) if the
        motion commands and reconfiguration are sent at the same time; so
        we just gate the motion commands when reconfiguring which should be OK in general
        since most result in zero velocity on the embedded side. Need to dig in further
        to understand why this is happening
        """
        self.parameter_server_is_updating = True
        rospy.sleep(0.1)
        
        """
        The first time through we want to ignore the values because they are just defaults from the ROS
        parameter server which has no knowledge of the platform being used
        """     
        if (True == self.is_init):
            self.is_init = False
            return config
    
        """
        Create the configuration bitmap from the appropriate variables
        """
        config_bitmap = (((config.motion_while_charging^1) << DISABLE_AC_PRESENT_CSI_SHIFT)|
                         (config.motion_ctl_input_filter << MOTION_MAPPING_FILTER_SHIFT))
        
        """
        Define the configuration parameters for all the platforms
        """
        self.valid_config_cmd  = [LOAD_MACH_CONFIG_CMD_ID,
                                  [convert_float_to_u32(config.x_vel_limit_mps),
                                   convert_float_to_u32(config.y_vel_limit_mps),
                                   convert_float_to_u32(config.accel_limit_mps2),
                                   convert_float_to_u32(config.decel_limit_mps2),
                                   convert_float_to_u32(config.dtz_decel_limit_mps2),
                                   convert_float_to_u32(config.yaw_rate_limit_rps),
                                   convert_float_to_u32(config.yaw_accel_limit_rps2),
                                   convert_float_to_u32(config.wheel_diameter_m),
                                   convert_float_to_u32(config.wheel_base_length_m),
                                   convert_float_to_u32(config.wheel_track_width_m),
                                   convert_float_to_u32(config.gear_ratio),
                                   config_bitmap]]
             
        """
        The teleop limits are always the minimum of the actual machine limit and the ones set for teleop
        """
        config.teleop_x_vel_limit_mps = minimum_f(config.teleop_x_vel_limit_mps, config.x_vel_limit_mps)
        config.teleop_y_vel_limit_mps = minimum_f(config.teleop_y_vel_limit_mps, config.x_vel_limit_mps)
        config.teleop_accel_limit_mps2 = minimum_f(config.teleop_accel_limit_mps2, config.accel_limit_mps2)
        config.teleop_yaw_rate_limit_rps = minimum_f(config.teleop_yaw_rate_limit_rps, config.yaw_rate_limit_rps)
        config.teleop_yaw_accel_limit_rps2 = minimum_f(config.teleop_yaw_accel_limit_rps2, config.teleop_yaw_accel_limit_rps2)      
        
        """
        Set the teleop configuration in the feedback
        """
        self.vector_data.config_param.SetTeleopConfig([config.teleop_x_vel_limit_mps,
                                                    config.teleop_y_vel_limit_mps,
                                                    config.teleop_accel_limit_mps2,
                                                    config.teleop_yaw_rate_limit_rps,
                                                    config.teleop_yaw_accel_limit_rps2])
                                                    
        
        """
        Define the configuration parameters for all the platforms
        """
        self.valid_pid_cmd  = [SET_PID_CONFIG_CMD_ID,
                                  [convert_float_to_u32(config.p_gain_rps_per_rps),
                                   convert_float_to_u32(config.i_gain_rps_per_rad),
                                   convert_float_to_u32(config.d_gain_rps_per_rps2),
                                   convert_float_to_u32(config.fdfwd_gain_rps_per_motor_rps),
                                   convert_float_to_u32(config.p_error_limit_rps),
                                   convert_float_to_u32(config.i_error_limit_rad),
                                   convert_float_to_u32(config.d_error_limit_rps2),
                                   convert_float_to_u32(config.i_error_drain_rate_rad_per_frame),
                                   convert_float_to_u32(config.output_limit_rps),
                                   convert_float_to_u32(config.input_target_limit_rps)]]
                                                    
        """
        Update the linear actuator velocity limit
        """
 
        
        """
        Check and see if we need to store the parameters in NVM before we try, although the NVM is F-RAM
        with unlimited read/write, uneccessarily setting the parameters only introduces risk for error 
        """
        load_config_params = False
        if self.param_server_initialized:
            if ((1<<7) == ((1<<7) & level)):
                self._linear.UpdateVelLimit(config.linear_actuator_vel_limit_mps)
            if ((1<<2) == ((1<<2) & level)):
                self._force_machine_configuration()
                load_config_params = True

            config.x_vel_limit_mps = round(self.vector_data.config_param.machcfg.x_vel_limit_mps,3)
            config.y_vel_limit_mps = round(self.vector_data.config_param.machcfg.y_vel_limit_mps,3)
            config.accel_limit_mps2 = round(self.vector_data.config_param.machcfg.accel_limit_mps2,3)
            config.decel_limit_mps2 = round(self.vector_data.config_param.machcfg.decel_limit_mps2,3)
            config.dtz_decel_limit_mps2 = round(self.vector_data.config_param.machcfg.dtz_decel_limit_mps2,3)
            config.yaw_rate_limit_rps = round(self.vector_data.config_param.machcfg.yaw_rate_limit_rps,3)
            config.yaw_accel_limit_rps2 = round(self.vector_data.config_param.machcfg.yaw_accel_limit_rps2,3)
            config.wheel_diameter_m = round(self.vector_data.config_param.machcfg.wheel_diameter_m,3)
            config.wheel_base_length_m = round(self.vector_data.config_param.machcfg.wheelbase_length_m,3)
            config.wheel_track_width_m = round(self.vector_data.config_param.machcfg.wheel_track_width_m,3)
            config.gear_ratio = round(self.vector_data.config_param.machcfg.gear_ratio,3)
            config.motion_while_charging = ((self.vector_data.config_param.machcfg.config_bitmap >> DISABLE_AC_PRESENT_CSI_SHIFT) & 1) ^ 1
            config.motion_ctl_input_filter = (self.vector_data.config_param.machcfg.config_bitmap >> MOTION_MAPPING_FILTER_SHIFT) & VALID_MOTION_MAPPING_FILTER_MASK
        

                                   
        if self.param_server_initialized:
            if ((1<<5) == ((1<<5) & level)):
                if self.vector_data.config_param.ctlconfig.control_tuning_unlocked:
                    self._force_pid_configuration()
                    
            if (True == config.set_default_gains):
                cmds = [GENERAL_PURPOSE_CMD_ID,[2002,0]]
                self._add_command_to_queue(cmds)
                config.set_default_gains = False
                rospy.sleep(0.1)

            config.p_gain_rps_per_rps = round(self.vector_data.config_param.ctlconfig.p_gain_rps_per_rps,3)
            config.i_gain_rps_per_rad = round(self.vector_data.config_param.ctlconfig.i_gain_rps_per_rad,3)
            config.d_gain_rps_per_rps2 = round(self.vector_data.config_param.ctlconfig.d_gain_rps_per_rps2,3)
            config.fdfwd_gain_rps_per_motor_rps = round(self.vector_data.config_param.ctlconfig.fdfwd_gain_rps_per_motor_rps,3)
            config.p_error_limit_rps = round(self.vector_data.config_param.ctlconfig.p_error_limit_rps,3)
            config.i_error_limit_rad = round(self.vector_data.config_param.ctlconfig.i_error_limit_rad,3)
            config.d_error_limit_rps2 = round(self.vector_data.config_param.ctlconfig.d_error_limit_rps2,3)
            config.i_error_drain_rate_rad_per_frame = round(self.vector_data.config_param.ctlconfig.i_error_drain_rate_rad_per_frame,5)
            config.output_limit_rps = round(self.vector_data.config_param.ctlconfig.output_limit_rps,3)
            config.input_target_limit_rps = round(self.vector_data.config_param.ctlconfig.input_target_limit_rps,3)

                
        if (True == config.send_unlock_request):
            try:
                key = int(config.unlock_key,16)
                self._unlock_gain_tuning(key)
            except:
                rospy.logerr("Invalid Key Value!!!! Must be a string in 32bit hex format") 
            config.send_unlock_request = False            
        
        
        self.valid_config = config
        
        if (True == load_config_params) or (False == self.param_server_initialized):
            self.update_base_local_planner = True
            self._update_move_base_params()
        
        self.param_server_initialized = True
        self.parameter_server_is_updating = False
        return config
        
    
    def _update_move_base_params(self):
        
        """
        If parameter updates have not been called in the last 5 seconds allow the
        subscriber callback to set them
        """
        if ((rospy.Time.now().to_sec()-self.last_move_base_update) > 5.0):
            self.update_base_local_planner = True
            
        if self.update_base_local_planner:
            self.update_base_local_planner = False
            self.last_move_base_update = rospy.Time.now().to_sec()
            
            try:
                dyn_reconfigure_client= Client("/move_base/DWAPlannerROS",timeout=1.0)
                changes = dict()
                changes['acc_lim_x'] = maximum_f(self.valid_config.accel_limit_mps2, self.valid_config.decel_limit_mps2)
                changes['acc_lim_y'] = maximum_f(self.valid_config.accel_limit_mps2, self.valid_config.decel_limit_mps2)
                changes['acc_lim_th'] = self.valid_config.yaw_accel_limit_rps2
                changes['max_vel_x'] = self.valid_config.x_vel_limit_mps
                changes['max_vel_y'] = self.valid_config.y_vel_limit_mps
                changes['max_rot_vel'] = self.valid_config.yaw_rate_limit_rps
                dyn_reconfigure_client.update_configuration(changes)
                dyn_reconfigure_client.close()
            except:
                pass
            
            
            rospy.loginfo("Vector Driver updated move_base parameters to match machine parameters")
            
    def _unlock_gain_tuning(self,key):
        r = rospy.Rate(10)
        attempts = 0
        while (False == self.vector_data.config_param.ctlconfig.control_tuning_unlocked) and (attempts < 3):
            
            cmds = [GENERAL_PURPOSE_CMD_ID,[2001,key]]
            self._add_command_to_queue(cmds)
            attempts += 1
            r.sleep()
        if (True == self.vector_data.config_param.ctlconfig.control_tuning_unlocked):
            rospy.loginfo("Controller tuning successfully unlocked")
        else:
            rospy.logerr("Failed to unlock controller tuning, the key is likely incorrect")
            
               

    def _continuous_data(self,start_cont):
        set_continuous = [GENERAL_PURPOSE_CMD_ID,[GENERAL_PURPOSE_CMD_SEND_CONTINUOUS_DATA,start_cont]]
        ret = False
        
        if (True == start_cont):
            r = rospy.Rate(10)
            start_time = rospy.Time.now().to_sec()
            while ((rospy.Time.now().to_sec() - start_time) < 3.0) and (True == self.vector_data.status.init):
                self._add_command_to_queue(set_continuous)
                r.sleep()
            ret = not self.vector_data.status.init
        else:
            r = rospy.Rate(5)
            start_time = rospy.Time.now().to_sec()
            while ((rospy.Time.now().to_sec() - start_time) < 3.0) and (False == ret):
                self._add_command_to_queue(set_continuous)
                rospy.sleep(0.6)
                if ((rospy.Time.now().to_sec() - self.last_rsp_rcvd) > 0.5):
                    ret = True
                r.sleep()
            self.vector_data.status.init = True

        return ret
    
    def _extract_faultlog(self):
        r = rospy.Rate(2)        
        start_time = rospy.Time.now().to_sec()
        while ((rospy.Time.now().to_sec() - start_time) < 3.0) and (True == self.extracting_faultlog):
            self._add_command_to_queue([GENERAL_PURPOSE_CMD_ID,[GENERAL_PURPOSE_CMD_SEND_FAULTLOG,0]]) 
            r.sleep()
            
        return not self.extracting_faultlog
    
    def _force_machine_configuration(self,echo=False):
        """
        Load all the parameters on the machine at startup; first check if they match, if they do continue.
        Otherwise load them and check again.
        """
        r = rospy.Rate(5)
        start_time = rospy.get_time()
        params_loaded = False
        new_params = False
        while ((rospy.get_time() - start_time) < 10.0) and (False == params_loaded):
            load_params = False
            for i in range(NUMBER_OF_CONFIG_PARAM_VARIABLES):
                if (self.vector_data.config_param.configuration_feedback[i] != self.valid_config_cmd[1][i]):
                    load_params = True
            if (True == load_params):
                self._add_command_to_queue(self.valid_config_cmd)
                new_params = True
                r.sleep()
            else:
                params_loaded = True
        
        if (new_params == True) or (echo == True):
            rospy.loginfo("New Machine Parameters Loaded.......")
            rospy.loginfo("x_vel_limit_mps:          %.4f"%self.vector_data.config_param.machcfg.x_vel_limit_mps)
            rospy.loginfo("y_vel_limit_mps:          %.4f"%self.vector_data.config_param.machcfg.y_vel_limit_mps)
            rospy.loginfo("accel_limit_mps2:         %.4f"%self.vector_data.config_param.machcfg.accel_limit_mps2)
            rospy.loginfo("decel_limit_mps2:         %.4f"%self.vector_data.config_param.machcfg.decel_limit_mps2)
            rospy.loginfo("dtz_decel_limit_mps2:     %.4f"%self.vector_data.config_param.machcfg.dtz_decel_limit_mps2)
            rospy.loginfo("yaw_rate_limit_rps:       %.4f"%self.vector_data.config_param.machcfg.yaw_rate_limit_rps)
            rospy.loginfo("yaw_accel_limit_rps2:     %.4f"%self.vector_data.config_param.machcfg.yaw_accel_limit_rps2)
            rospy.loginfo("wheel_diameter_m:         %.4f"%self.vector_data.config_param.machcfg.wheel_diameter_m)
            rospy.loginfo("wheel_base_length_m:      %.4f"%self.vector_data.config_param.machcfg.wheelbase_length_m)
            rospy.loginfo("wheel_track_width_m:      %.4f"%self.vector_data.config_param.machcfg.wheel_track_width_m)
            rospy.loginfo("gear_ratio:               %.4f"%self.vector_data.config_param.machcfg.gear_ratio)
            rospy.loginfo("motion_while_charging:    %u"%(((self.vector_data.config_param.machcfg.config_bitmap >> DISABLE_AC_PRESENT_CSI_SHIFT) & 1) ^ 1))
            rospy.loginfo("motion_ctl_input_filter:  %u\n"%((self.vector_data.config_param.machcfg.config_bitmap >> MOTION_MAPPING_FILTER_SHIFT) & VALID_MOTION_MAPPING_FILTER_MASK))
        elif not params_loaded:
            rospy.logerr("Failed to load machine parameters!!!!!!!!!!")
            
        return params_loaded

    def _force_pid_configuration(self,echo=False):                
        r = rospy.Rate(5)
        start_time = rospy.get_time()
        params_loaded = False
        new_params = False
        while ((rospy.get_time() - start_time) < 10.0) and (False == params_loaded):
            load_params = False
            for i in range(16,(16+NUMBER_OF_PID_PARAM_VARIABLES)):
                if (self.vector_data.config_param.configuration_feedback[i] != self.valid_pid_cmd[1][i-16]):
                    load_params = True

            if (True == load_params):
                self._add_command_to_queue(self.valid_pid_cmd)
                r.sleep()
                new_params = True
            else:
                params_loaded = True
                
        if (new_params == True) or (echo == True):                
            rospy.loginfo("New PID Parameters Loaded.......")
            rospy.loginfo("p_gain_rps_per_rps:                %.4f"%self.vector_data.config_param.ctlconfig.p_gain_rps_per_rps)
            rospy.loginfo("i_gain_rps_per_rps:                %.4f"%self.vector_data.config_param.ctlconfig.i_gain_rps_per_rad)
            rospy.loginfo("d_gain_rps_per_rps:                %.4f"%self.vector_data.config_param.ctlconfig.d_gain_rps_per_rps2)
            rospy.loginfo("fdfwd_gain_rps_per_motor_rps:      %.4f"%self.vector_data.config_param.ctlconfig.fdfwd_gain_rps_per_motor_rps)
            rospy.loginfo("p_error_limit_rps:                 %.4f"%self.vector_data.config_param.ctlconfig.p_error_limit_rps)
            rospy.loginfo("i_error_limit_rps:                 %.4f"%self.vector_data.config_param.ctlconfig.i_error_limit_rad)
            rospy.loginfo("d_error_limit_rps:                 %.4f"%self.vector_data.config_param.ctlconfig.d_error_limit_rps2)
            rospy.loginfo("i_error_drain_rate_rad_per_frame:  %.4f"%self.vector_data.config_param.ctlconfig.i_error_drain_rate_rad_per_frame)
            rospy.loginfo("output_limit_rps:                  %.4f"%self.vector_data.config_param.ctlconfig.output_limit_rps)
            rospy.loginfo("input_target_limit_rps:            %.4f\n"%self.vector_data.config_param.ctlconfig.input_target_limit_rps)
        elif not params_loaded:
            rospy.logerr("Failed to load machine parameters!!!!!!!!!!")        
        return params_loaded
    
