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
 
 \file   jaco_joint_controller.py

 \brief  This module contains a collection of functions low level interface
         to the Kinova API.

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from ctypes import *
import rospy
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import threading
import math
from angles import *
from helpers import limit,RateLimitSignals,FilterSignals,DifferentiateSignals
from jaco_joint_pid import JacoPID
import operator

class KinovaDevice(Structure):
    _fields_=[("SerialNumber",c_char*20),
              ("Model",c_char*20),
              ("VersionMajor",c_int),
              ("VersionMinor",c_int),
              ("VersionRelease",c_int),
              ("DeviceType",c_int),
              ("DeviceID",c_int)]
    
class AngularInfo(Structure):
    _fields_=[("Actuator1",c_float),
              ("Actuator2",c_float),
              ("Actuator3",c_float),
              ("Actuator4",c_float),
              ("Actuator5",c_float),
              ("Actuator6",c_float)]
class FingersPosition(Structure):
    _fields_=[("Finger1",c_float),
              ("Finger2",c_float),
              ("Finger3",c_float)]
    
class AngularPosition(Structure):
    _fields_=[("Actuators",AngularInfo),
              ("Fingers",FingersPosition)]
    
class Limitation(Structure):
    _fields_ = [("speedParameter1",c_float),
                ("speedParameter2",c_float),
                ("speedParameter3",c_float),
                ("forceParameter1",c_float),
                ("forceParameter2",c_float),
                ("forceParameter3",c_float),
                ("accelerationParameter1",c_float),
                ("accelerationParameter2",c_float),
                ("accelerationParameter3",c_float)]
    
class CartesianInfo(Structure):
    _fields_ = [("X",c_float),
                ("Y",c_float),
                ("Z",c_float),
                ("ThetaX",c_float),
                ("ThetaY",c_float),
                ("ThetaZ",c_float)]
    
class UserPosition(Structure):
    _fields_ = [("Type",c_int),
                ("Delay",c_float),
                ("CartesianPosition",CartesianInfo),
                ("Actuators",AngularInfo),
                ("HandMode",c_int),
                ("Fingers",FingersPosition)]
    
class TrajectoryPoint(Structure):
    _fields_ = [("Position",UserPosition),
                ("LimitationsActive",c_int),
                ("SynchroType",c_int),
                ("Limitations",Limitation)]   

NO_ERROR_KINOVA = 1
DEG_TO_RAD = (math.pi / 180.0)
RAD_TO_DEG = (180.0 / math.pi)
ANGULAR_POSITION = 2
ANGULAR_VELOCITY = 8
LARGE_ACTUATOR_VELOCITY =deg_to_rad(35.0) #maximum velocity of large actuator (joints 1-3) (deg/s)
SMALL_ACTUATOR_VELOCITY =deg_to_rad(45.0) #maximum velocity of small actuator (joints 4-6) (deg/s)

JOINT_VEL_LIMITS = [LARGE_ACTUATOR_VELOCITY,
                    LARGE_ACTUATOR_VELOCITY,
                    LARGE_ACTUATOR_VELOCITY,
                    SMALL_ACTUATOR_VELOCITY,
                    SMALL_ACTUATOR_VELOCITY,
                    SMALL_ACTUATOR_VELOCITY]
        
class SIArmController(object):
    def __init__(self, joint_names, arm, serial_num=""):
        
        """
        Assume success until posted otherwise
        """
        self.init_success = True

        """
        Setup a lock for accessing data in the control loop
        """
        self._lock = threading.Lock()
        
        """
        Make sure the naming conventions and number of joints match what we expect
        """
        self._joint_names = joint_names
        self._num_joints = len(self._joint_names)
        if (len(self._joint_names) != 6) and (len(self._joint_names) != 4):
            rospy.logerr("Kinova products only support 4 or 6 DOF arms, caller passed %d...ABORTING"%len(self._joint_names))
            self.init_success = False
            return

        self._arm_name = arm
        if (self._arm_name != 'right') and (self._arm_name != 'left'):
            rospy.logerr("Must define arm as left or right, caller passed %s...ABORTING"%self._arm_name)
            self.init_success = False
            return
             
        """
        Create the hooks for the API
        """
        self.kinova=CDLL('Kinova.API.USBCommandLayerUbuntu.so')
        self.InitAPI = self.kinova.InitAPI
        self.CloseAPI = self.kinova.CloseAPI
        self.GetDevices = self.kinova.GetDevices
        self.GetDevices.argtypes = [POINTER(KinovaDevice),POINTER(c_int)]
        self.SetActiveDevice = self.kinova.SetActiveDevice
        self.SetActiveDevice.argtypes = [KinovaDevice]
        self.GetAngularPosition = self.kinova.GetAngularPosition
        self.GetAngularPosition.argtypes = [POINTER(AngularPosition)]
        self.GetAngularVelocity = self.kinova.GetAngularVelocity
        self.GetAngularVelocity.argtypes = [POINTER(AngularPosition)]
        self.GetAngularForce = self.kinova.GetAngularForce
        self.GetAngularForce.argtypes = [POINTER(AngularPosition)]
        self.StartControlAPI = self.kinova.StartControlAPI
        self.SetAngularControl = self.kinova.SetAngularControl
        self.StopControlAPI = self.kinova.StopControlAPI
        self.SendAdvanceTrajectory = self.kinova.SendAdvanceTrajectory
        self.SendAdvanceTrajectory.argtypes = [TrajectoryPoint]
        self.SendBasicTrajectory = self.kinova.SendBasicTrajectory
        self.SendBasicTrajectory.argtypes = [TrajectoryPoint]
        self.EraseAllTrajectories = self.kinova.EraseAllTrajectories
        self.SetAngularControl = self.kinova.SetAngularControl
        self.DevInfoArrayType = ( KinovaDevice * 20 )
        
        """
        Let the API try a few times to list devices, they take a few seconds to come up and
        sometimes the API can fail if the caller hasn't waited for the hardware to come up
        """
        attempts = 0
        success = False  
        while not success and (attempts < 5) and not rospy.is_shutdown():
            result1 = self.InitAPI()
            result2 = c_int(0)
            devinfo = self.DevInfoArrayType()
            num_arms = self.GetDevices(devinfo,byref(result2))
            if (NO_ERROR_KINOVA == result1) and (NO_ERROR_KINOVA == result2.value) and (num_arms > 0):
                success = True
            else:
                rospy.logwarn("API failed to initialize.....Trying again.....")
                self.CloseAPI()
                attempts+=1
                rospy.sleep(2)

        if not success:
            self.init_success = False
            rospy.logerr("Init API result:   %d"%result1) 
            rospy.logerr("GetDevices result: %d"%result2)
            rospy.logerr("Number of arms:    %d"%num_arms)
            rospy.logerr("Initialization failed, could not find Kinova devices \
                          (see Kinova.API.CommLayerUbuntu.h for details)")
            self.CloseAPI()
            return
        
        """
        Make sure there are devices available and that there is a device with the
        serial number we are looking for
        """
        found_arm = False
        other_arms = []
        for i in range(num_arms):
            if (devinfo[i].SerialNumber == serial_num):
                self._arm = devinfo[i]
                rospy.loginfo("%s arm has serial number: %s"%(self._arm_name,str(self._arm.SerialNumber)))
                found_arm = True
            else:
                other_arms.append(str(devinfo[i].SerialNumber))
        if not found_arm and (''==serial_num):
            rospy.logwarn("No serial number passed, using the first device in the list...")
            self._arm = devinfo[0]
            other_arms.pop(0)
            rospy.loginfo("%s arm has serial number: %s"%(self._arm_name,str(self._arm.SerialNumber)))
            found_arm = True
                    
        if (len(other_arms) > 0):
            rospy.logwarn("Found other arms not matching serial number parameter:")
        for sn in other_arms:
            rospy.logwarn(sn)
            
        if not found_arm:
            self.init_success = False
            rospy.logerr("Could not find %s arm with serial number %s"%(self._arm_name,serial_num))
            rospy.logerr("Initialization failed; did not connect to desired arm...stopping driver")
            self.CloseAPI()
            self.init_failed = True
            return

        """
        Try and set the active device, the API version not matching is usually why this fails
        """
        result = self.SetActiveDevice(self._arm)
        if not (NO_ERROR_KINOVA == result):
            rospy.logerr("Could not set %s arm active...stopping the driver"%self._arm_name)
            rospy.logerr("Set Active Device result:   %d"%result) 
            rospy.logerr("Initialization failed, could not find Kinova devices \
                          (see Kinova.API.CommLayerUbuntu.h for details)")
            self.StopControlAPI()
            self.CloseAPI()
            self.init_success = False
            return
        self._position_hold = False
              
        """
        Initialize the PID controllers
        """
        self._pid = [None]*self._num_joints
        self._pid[0] = JacoPID(0.0,0.0,0.0)
        self._pid[1] = JacoPID(0.0,0.0,0.0)
        self._pid[2] = JacoPID(0.0,0.0,0.0)
        self._pid[3] = JacoPID(0.0,0.0,0.0)
        self._pid[4] = JacoPID(0.0,0.0,0.0)
        #self._pid[5] = JacoPID(17.5,0.0,2.1)
        #self._pid[5] = JacoPID(19.5,0.0,2.8)
        self._pid[5] = JacoPID(17.5,0.0,2.1) 
        self._pid_error = [0.0]*self._num_joints
        self._pid_output = [0.0]*self._num_joints
        
        """
        Initialize the command interface, data processing and controller data
        """                  
        self._raw_cmds = dict()
        self._raw_cmds['position'] = self._get_angular_position()
        self._raw_cmds['velocity'] = [0.0]*self._num_joints
        self._raw_cmds['acceleration'] = [0.0]*self._num_joints

        self._processed_cmds = dict()
        self._processed_cmds['target'] = self._get_angular_position()
        self._processed_cmds['rl_target'] = self._get_angular_position()
        self._processed_cmds['ff_vel'] = [0.0]*self._num_joints
        self._processed_cmds['ff_acc'] = [0.0]*self._num_joints

        self._pid_input_filters = dict()
        self._pid_input_filters['target'] = FilterSignals(0.5,self._num_joints,self._get_angular_position())
        self._pid_input_filters['ff_vel'] = FilterSignals(2.0,self._num_joints,[0.0]*self._num_joints)
        self._pid_input_filters['ff_acc'] = FilterSignals(2.0,self._num_joints,[0.0]*self._num_joints)
        self._pid_input_filters['rl_target'] = RateLimitSignals(JOINT_VEL_LIMITS,self._num_joints,self._get_angular_position())
        
        self._vel_diff = DifferentiateSignals(self._num_joints, self._get_angular_position())
        self._vff_diff = DifferentiateSignals(self._num_joints, self._get_angular_position())

        self._joint_fb = dict()
        self._joint_fb['position'] = self._get_angular_position()
        self._joint_fb['velocity'] = self._get_angular_velocity()
        self._joint_fb['force'] = self._get_angular_force()

        self._joint_fb_filters = dict()
        self._joint_fb_filters['position'] = FilterSignals(2.0,self._num_joints,self._get_angular_position())
        self._joint_fb_filters['velocity'] = FilterSignals(2.0,self._num_joints,[0.0]*self._num_joints)
        self._joint_fb_filters['force'] = FilterSignals(2.0,self._num_joints,self._get_angular_force())

        """
        Register the publishers and subscribers
        """
        self._jstpub = rospy.Publisher("/vector/%s_arm_controller/state"%self._arm_name,JointTrajectoryControllerState,queue_size=10)
        self._jstmsg = JointTrajectoryControllerState()
        self._jstmsg.header.seq = 0
        self._jstmsg.header.frame_id = ''
        self._jstmsg.header.stamp = rospy.get_rostime() 
        self._jspub = rospy.Publisher("/vector/%s_arm/joint_states"%self._arm_name,JointState,queue_size=10)
        self._jsmsg = JointState()
        self._jsmsg.header.seq = 0
        self._jsmsg.header.frame_id = ''
        self._jsmsg.header.stamp = rospy.get_rostime()
        self._jsmsg.name  = self._joint_names
        
        """
        Update the feedback once to get things initialized
        """
        self._update_controller_data()
        
        """
        Start the controller
        """ 
        rospy.loginfo("Starting the %s controller"%arm)
        self._done = False
        self._t1 = rospy.Timer(rospy.Duration(0.01),self._run_ctl)

    def Stop(self):
        rospy.loginfo("Stopping the %s arm controller"%self._arm_name)
        with self._lock:
            try:
                self._t1.shutdown()
            except:
                pass
            try:
                self._jspub.unregister()
            except:
                pass
            self.StopControlAPI()
            self.CloseAPI()
            
            rospy.loginfo("%s arm controller has stopped"%self._arm_name)
            self._done = True

    def _is_shutdown(self):
        if rospy.is_shutdown():
            self.Stop()
        return self._done
        
    def UpdatePIDGains(self,pid_gains):
    
        new_pid_gains = [pid_gains[jnt] for jnt in self._joint_names]
        
    def SetPositionHold(self):
        if self._position_hold:
            return
        with self._lock:
            self._position_hold=True
            self._raw_cmds['position'] = self._get_angular_position()
            self._raw_cmds['velocity'] = [0.0]*self._num_joints
            self._raw_cmds['acceleration'] = [0.0]*self._num_joints
            
    def ClearPositionHold(self):
        with self._lock:
            self._position_hold=False
        
    def CommandJoints(self,pos,vel=None,acc=None):
        if self._position_hold:
            return False
            
        with self._lock:
            self._raw_cmds['position'] = [pos[jnt] for jnt in self._joint_names]
            if vel:
                self._raw_cmds['velocity'] = [vel[jnt] for jnt in self._joint_names]
            else:
                self._raw_cmds['velocity'] = [0.0]*self._num_joints    
            if acc:
                self._raw_cmds['acceleration'] = [acc[jnt] for jnt in self._joint_names]
            else:
                self._raw_cmds['acceleration'] = [0.0]*self._num_joints
        return True            

    def GetCurrentJointPosition(self, joint_names):
        with self._lock:
            pos = dict(zip(self._jsmsg.name,self._joint_fb['position']))
        pos = [pos[jnt] for jnt in joint_names]
        return pos
        
    def GetCurrentJointVelocity(self,joint_names):
        with self._lock:
            vel = dict(zip(self._jsmsg.name,self._joint_fb['velocity']))
        vel = [vel[jnt] for jnt in joint_names]
        return vel

    def GetCurrentJointPositionError(self,joint_names):
        with self._lock:
            pos_error = dict(zip(self._jsmsg.name,self._pid_error))
        pos_error = [pos_error[jnt] for jnt in joint_names]
        return pos_error  
         
    def _update_controller_data(self):
        self._joint_fb['position'] = self._joint_fb_filters['position'].Update(self._get_angular_position())
        self._joint_fb['velocity'] = self._joint_fb_filters['velocity'].Update(self._get_angular_velocity())
        self._joint_fb['force'] = self._joint_fb_filters['force'].Update(self._get_angular_force())

        self._jsmsg.header.stamp = rospy.get_rostime()
        self._jsmsg.position = self._joint_fb['position']
        self._jsmsg.velocity = self._joint_fb['velocity']
        self._jsmsg.effort = self._joint_fb['force']
        self._jspub.publish(self._jsmsg)
        self._jsmsg.header.seq+=1

    def _get_angular_position(self):
        pos = AngularPosition()
        self.GetAngularPosition(byref(pos))
        tmp = [deg_to_rad(pos.Actuators.Actuator1),
               deg_to_rad(pos.Actuators.Actuator2-180.0),
               deg_to_rad(pos.Actuators.Actuator3-180.0),
               deg_to_rad(pos.Actuators.Actuator4),
               deg_to_rad(pos.Actuators.Actuator5),
               deg_to_rad(pos.Actuators.Actuator6)]
               
        ret = [tmp[i] for i in range(self._num_joints)]
        return ret

    def _get_angular_velocity(self):
        """
        Velocity reported by API is incorrect so differentiate the position feedback
        """
        return self._vel_diff.Update(self._get_angular_position())
        
    def _get_angular_force(self):
        force = AngularPosition()
        self.GetAngularForce(byref(force))
        tmp = [force.Actuators.Actuator1,
               force.Actuators.Actuator2,
               force.Actuators.Actuator3,
               force.Actuators.Actuator4,
               force.Actuators.Actuator5,
               force.Actuators.Actuator6]
               
        ret = [tmp[i] for i in range(self._num_joints)]
        
        return ret
    
    def _run_ctl(self,events):
        if self._is_shutdown():
            return
        
        with self._lock:
            """
            First update the controller data
            """
            self._update_controller_data()
            
            """
            Compute the error and update the feedforward terms
            """
            self._processed_cmds['rl_target'] = self._pid_input_filters['rl_target'].Update(self._raw_cmds['position']) 
            self._processed_cmds['target'] = self._pid_input_filters['target'].Update(self._processed_cmds['rl_target'])
            self._processed_cmds['ff_acc'] = self._pid_input_filters['ff_acc'].Update(self._raw_cmds['acceleration'])
            vff = self._vff_diff.Update(self._processed_cmds['target'])
            self._processed_cmds['ff_vel'] = self._pid_input_filters['ff_vel'].Update(self._raw_cmds['velocity'])
            """
            For now gains are just zero
            """
            scaled_ff_vel = map(operator.mul, self._processed_cmds['ff_vel'], [1.0] * self._num_joints)
            scaled_ff_acc = map(operator.mul, self._processed_cmds['ff_acc'], [0.0] * self._num_joints)
            ff_terms = map(operator.add, scaled_ff_vel, scaled_ff_acc)
            
            self._pid_error =  map(operator.sub, self._processed_cmds['target'], self._joint_fb['position'])
            self._pid_output = [self._pid[i].compute_output(self._pid_error[i]) for i in range(self._num_joints)]
            self._pid_output = map(operator.add, self._pid_output, ff_terms)
            self._pid_output = [rad_to_deg(limit(self._pid_output[i],JOINT_VEL_LIMITS[i])) for i in range(self._num_joints)]

            
            """
            override all the joints except the wrist for debug
            """
            tmp = self._pid_output[self._num_joints-1]
            self._pid_output = [0.0]*self._num_joints
            self._pid_output[self._num_joints-1] = tmp
                
            """
            Send the command via the API
            """
            traj = TrajectoryPoint()
            traj.Position.Type = ANGULAR_VELOCITY
            traj.Position.Actuators.Actuator1 = self._pid_output[0]
            traj.Position.Actuators.Actuator2 = self._pid_output[1]
            traj.Position.Actuators.Actuator3 = self._pid_output[2]
            traj.Position.Actuators.Actuator4 = self._pid_output[3]
            traj.Position.Actuators.Actuator5 = self._pid_output[4]
            traj.Position.Actuators.Actuator6 = self._pid_output[5]
            self.SendBasicTrajectory(traj)
            
            #print self._processed_cmds['target'][5]
            
            """
            Publish the controller state
            """    
            self._jstmsg.header.frame_id = ''
            self._jstmsg.header.stamp = rospy.get_rostime()
            self._jstmsg.desired.positions=self._processed_cmds['target']
            self._jstmsg.desired.velocities=self._processed_cmds['ff_vel']
            self._jstmsg.desired.accelerations=self._processed_cmds['ff_acc']
            self._jstmsg.actual.positions=self._joint_fb['position']
            self._jstmsg.actual.velocities=self._joint_fb['velocity']
            self._jstmsg.actual.accelerations=[0.0]*self._num_joints
            self._jstmsg.error.positions = self._pid_error
            self._jstmsg.error.velocities= map(operator.sub, self._processed_cmds['ff_vel'], self._joint_fb['velocity']) 
            self._jstmsg.error.accelerations=[0.0]*self._num_joints                
            self._jstpub.publish(self._jstmsg) 
            self._jstmsg.header.seq +=1                       
     
