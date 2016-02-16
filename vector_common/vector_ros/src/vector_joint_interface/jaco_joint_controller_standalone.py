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
from vector_msgs.msg import JacoJointCmd
from control_msgs.msg import JointTrajectoryControllerState
import threading
import math
from angles import *
from helpers import limit,RateLimitSignals,FilterSignals
from jaco_joint_pid import JacoPID

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
LARGE_ACTUATOR_VELOCITY =deg_to_rad(30.0) #maximum velocity of large actuator (joints 1-3) (deg/s)
SMALL_ACTUATOR_VELOCITY =deg_to_rad(35.0) #maximum velocity of small actuator (joints 4-6) (deg/s)

JOINT_VEL_LIMITS = [LARGE_ACTUATOR_VELOCITY,
                    LARGE_ACTUATOR_VELOCITY,
                    LARGE_ACTUATOR_VELOCITY,
                    SMALL_ACTUATOR_VELOCITY,
                    SMALL_ACTUATOR_VELOCITY,
                    SMALL_ACTUATOR_VELOCITY]


        
        
class Jaco2Py:
    def __init__(self,num_arms=1,serial_nums=["",""],vel_cmd_timeout=0.1):
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
        devinfo = self.DevInfoArrayType()
        num_dev = c_int(0)
        self.init_success = True

        r = rospy.Rate(0.2)
        attempts = 0
        result1 = self.InitAPI()            
        result2 = self.GetDevices(devinfo,byref(num_dev))
        while ((NO_ERROR_KINOVA != result1) or (NO_ERROR_KINOVA != result2)) and (attempts < 5):
            self.CloseAPI()
            r.sleep()
            rospy.logwarn("API indicates Kinova devices not ready.....Trying again.....")
            result1 = self.InitAPI()
            result2 = self.GetDevices(devinfo,byref(num_dev))
            attempts+=1

        if (NO_ERROR_KINOVA != result1) or (NO_ERROR_KINOVA != result2) or (attempts >= 5):
            self.init_success = False
            rospy.logerr("Init API result:   %d"%result1) 
            rospy.logerr("GetDevices result: %d"%result2)
            rospy.logerr("Initialization failed, could not find Kinova devices \
                          (see Kinova.API.CommLayerUbuntu.h for details)")
            self.CloseAPI()
            return
        
        if (num_dev.value < num_arms):
            self.init_success = False
            rospy.logerr("Not enough Kinova robotic arms found only found %d"%num_dev.value)
            self.CloseAPI()
            return
        elif (num_dev.value > num_arms):
            self.init_success = False
            rospy.logerr("Too many Kinova robotic arms found %d"%num_dev.value)
            self.CloseAPI()
            return
        else:
            rospy.loginfo("SUCCESS!!!! %d Kinova arms found"%num_arms)
            
        self.arms = dict()
        self.arm_idx = dict({0:'right',1:'left'})
        init_failed = False
        for arm in range(num_arms):
            arm_name = self.arm_idx[arm]
            if not ("" == serial_nums[arm]):
                found_arm = False
                for i in range(num_arms):
                    if (devinfo[i].SerialNumber == serial_nums[arm]):
                        self.arms[arm_name] = devinfo[i]
                        found_arm = True
                if (True == found_arm):
                    rospy.loginfo("%s arm has serial number: %s"%(arm_name,str(self.arms[arm_name].SerialNumber)))
                else:
                    rospy.logerr("Could not find %s arm with serial number %s"%(arm_name,str(self.arms[arm_name].SerialNumber)))
                    init_failed = True
            else:
                self.arms[arm_name] = devinfo[arm]
                rospy.loginfo("%s arm has serial number: %s"%(arm_name,str(self.arms[arm_name].SerialNumber)))
        
        if init_failed:
            rospy.logerr("Initialization failed; did not connect to all arms...stopping driver")
            self.CloseAPI()
            self.init_success = False
            return
        

        self.num_arms = num_arms
        self._pid = dict()
        self._jspubs = dict()
        self._jsmsgs = dict()
        self._jstpubs = dict()
        self._jstmsgs = dict()
        self._cmdsubs = dict()
        self._cmds = dict()
        self._last_jpos = dict()
        self._last_js_time = dict()
        self._prev_ff_vel = dict()
        self._fb_filtered = dict()
        self._tar_filtered = dict()
        self._tar_rl = dict()
        self._prev_tar = dict()
        self._lock = threading.Lock()
        for i in range(self.num_arms):
            arm_name = self.arm_idx[i]
            result = self.SetActiveDevice(self.arms[arm_name])
            if not (NO_ERROR_KINOVA == result):
                rospy.logerr("Could not set %s arm active...stopping the driver"%arm_name)
                self.StopControlAPI()
                self.CloseAPI()
                self.init_success = False
                break

                                     
            self._pid[arm_name] = [None]*6
            self._pid[arm_name][0] = JacoPID(0.0,0.0,0.0)
            self._pid[arm_name][1] = JacoPID(0.0,0.0,0.0)
            self._pid[arm_name][2] = JacoPID(0.0,0.0,0.0)
            self._pid[arm_name][3] = JacoPID(0.0,0.0,0.0)
            self._pid[arm_name][4] = JacoPID(0.0,0.0,0.0)
            #self._pid[arm_name][5] = JacoPID(17.5,0.0,2.1)
            #self._pid[arm_name][5] = JacoPID(19.5,0.0,2.8)
            self._pid[arm_name][5] = JacoPID(19.5,0.0,2.8)                   
            self._cmds[arm_name] = self._get_angular_position()
            self._last_jpos[arm_name] = self._get_angular_position(True)
            self._prev_tar[arm_name] = self._get_angular_position(True)
            
            jlims = JOINT_VEL_LIMITS
            jlims[5]-=0.05
            self._tar_rl[arm_name] = RateLimitSignals(jlims,6,self._get_angular_position(True))
            self._last_js_time[arm_name] = rospy.get_time()
            self._prev_ff_vel[arm_name] = [0.0]*6
            self._fb_filtered[arm_name] = FilterSignals(4.0,6,self._get_angular_position(True))
            self._tar_filtered[arm_name] = FilterSignals(0.5,6,self._get_angular_position(True)) 

            """
            Register the publishers and subscribers
            """
            JointTrajectoryControllerState
            self._jstpubs[arm_name] = rospy.Publisher("/vector/%s_arm_controller/state"%arm_name,JointTrajectoryControllerState,queue_size=10)
            self._jstmsgs[arm_name] = JointTrajectoryControllerState()
            self._jstmsgs[arm_name].header.seq = 0
            self._jstmsgs[arm_name].header.frame_id = ''
            self._jstmsgs[arm_name].header.stamp = rospy.get_rostime() 
            self._jspubs[arm_name] = rospy.Publisher("/vector/%s_arm/joint_states"%arm_name,JointState,queue_size=10)
            self._jsmsgs[arm_name] = JointState()
            self._jsmsgs[arm_name].header.seq = 0
            self._jsmsgs[arm_name].header.frame_id = ''
            self._jsmsgs[arm_name].header.stamp = rospy.get_rostime()
            
            
            j_names = ['']*6
            j_names[0] = '%s_shoulder_pan_joint'%arm_name
            j_names[1] = '%s_shoulder_lift_joint'%arm_name
            j_names[2] = '%s_elbow_joint'%arm_name
            j_names[3] = '%s_wrist_1_joint'%arm_name
            j_names[4] = '%s_wrist_2_joint'%arm_name
            j_names[5] = '%s_wrist_3_joint'%arm_name
            self._jsmsgs[arm_name].name=j_names

            if (arm_name == 'right'):
                self._cmdsubs['right'] = rospy.Subscriber("/vector/right_arm/joint_cmd",JacoJointCmd,self._update_right_joint_cmds)
            elif (arm_name == 'left'):
                self._cmdsubs['left'] = rospy.Subscriber("/vector/left_arm/joint_cmd",JacoJointCmd,self._update_left_joint_cmds)

        if True == self.init_success:
            rospy.loginfo("Starting the controller thread")
            self._done = False
            self._last_loop_time = rospy.get_time()
            self._t1 = rospy.Timer(rospy.Duration(0.01),self._pub_js)
            self._t2 = rospy.Timer(rospy.Duration(0.01),self._run_ctl)

    def _stop(self):
        rospy.loginfo("Stopping the controller thread")
        with self._lock:
            self._t1.shutdown()
            self._t2.shutdown()  
            self.StopControlAPI()
            self.CloseAPI()
            for i in range(self.num_arms):
                arm= self.arm_idx[i]
                self._cmdsubs[arm].unregister()
                self._jspubs[arm].unregister()
            rospy.loginfo("Controller thread has stopped")
            self._done = True

    def done(self):
        if rospy.is_shutdown():
            self._stop()
        return self._done           
            
    def _pub_js(self,tmp):
        with self._lock:
            for i in range(self.num_arms):
                arm= self.arm_idx[i]
                result = self.SetActiveDevice(self.arms[arm])
                if not (NO_ERROR_KINOVA == result):
                    rospy.logerr("Could not set %s arm active...stopping the driver"%arm)
                    break
                self._jsmsgs[arm].header.stamp = rospy.get_rostime()
                self._jsmsgs[arm].position = self._get_angular_position()
                self._jsmsgs[arm].velocity = self._get_angular_velocity(arm)
                self._jsmsgs[arm].effort = self._get_angular_force()
                self._jspubs[arm].publish(self._jsmsgs[arm])
                self._jsmsgs[arm].header.seq+=1
    
    def _update_right_joint_cmds(self,cmds):
        self._update_joint_cmds(cmds,'right')

    def _update_left_joint_cmds(self,cmds):
        self._update_joint_cmds(cmds,'left')
        
    def _update_joint_cmds(self,cmds,arm):
        with self._lock:
            try:
                self._cmds[arm] = [wrap_angle(cmds.joint_cmds[i]) for i in range(6)]
            except:
                rospy.logerr("Platform does not have a %s arm registered with the driver"%arm)
                return
    
    def _get_angular_position(self,raw=False):
        pos = AngularPosition()
        self.GetAngularPosition(byref(pos))
        ret = [0.0]*6
        if not raw:
            ret[0] = wrap_angle(deg_to_rad(pos.Actuators.Actuator1))
            ret[1] = wrap_angle(deg_to_rad(pos.Actuators.Actuator2-180.0))
            ret[2] = wrap_angle(deg_to_rad(pos.Actuators.Actuator3-180.0))
            ret[3] = wrap_angle(deg_to_rad(pos.Actuators.Actuator4))
            ret[4] = wrap_angle(deg_to_rad(pos.Actuators.Actuator5))
            ret[5] = wrap_angle(deg_to_rad(pos.Actuators.Actuator6))
        else:
            ret[0] = deg_to_rad(pos.Actuators.Actuator1)
            ret[1] = deg_to_rad(pos.Actuators.Actuator2-180.0)
            ret[2] = deg_to_rad(pos.Actuators.Actuator3-180.0)
            ret[3] = deg_to_rad(pos.Actuators.Actuator4)
            ret[4] = deg_to_rad(pos.Actuators.Actuator5)
            ret[5] = deg_to_rad(pos.Actuators.Actuator6)
        
        return ret

    def _get_angular_velocity(self,arm='right'):                      
        pos = self._get_angular_position(True)
        dt = rospy.get_time() - self._last_js_time[arm]
        self._last_js_time[arm] = rospy.get_time()
        vels = [(pos[i]-self._last_jpos[arm][i])/dt for i in range(6)]
        self._last_jpos[arm] = pos
        
        return vels
        
    def _get_angular_force(self):
        force = AngularPosition()
        self.GetAngularForce(byref(force))
        ret = [0.0]*6
        ret[0] = force.Actuators.Actuator1
        ret[1] = force.Actuators.Actuator2
        ret[2] = force.Actuators.Actuator3
        ret[3] = force.Actuators.Actuator4
        ret[4] = force.Actuators.Actuator5
        ret[5] = force.Actuators.Actuator6
        
        return ret
    
    def _run_ctl(self,events):
        with self._lock:
            for i in range(self.num_arms):
                arm = self.arm_idx[i]
                result = self.SetActiveDevice(self.arms[arm])
                if not (NO_ERROR_KINOVA == result):
                    rospy.logerr("Could not set %s arm active...stopping the driver"%arm)
                    break
                    
                
                fb = self._fb_filtered[arm].Update(self._get_angular_position(True))                  
                
                diff = [get_smallest_difference_to_cont_angle(self._cmds[arm][i],fb[i]) for i in range(6)]
                tmp = self._tar_rl[arm].Update([fb[i]+diff[i] for i in range(6)])
                tar = self._tar_filtered[arm].Update(tmp)
                dt = rospy.get_time()-self._last_loop_time
                self._last_loop_time = rospy.get_time()
                
                ff_rate = [0.0]*6 
                if (dt > 0.0):
                    ff_rate = [((tar[i]-self._prev_tar[arm][i])/dt) * 0.95 for i in range(6)]
                self._prev_tar[arm] = tar
               
                ff_acc = [0.0]*6
                #if (dt > 0.0):
                    #ff_acc = [-((self._prev_ff_vel[arm][i]- ff_rate[i])/dt) for i in range(6)]
                #self._prev_ff_vel[arm] = ff_rate

                error =  [tar[i]-fb[i] for i in range(6)]
                output = [self._pid[arm][i].compute_output(error[i]) for i in range(6)]
                output = [output[i] + ff_rate[i] + ff_acc[i] for i in range(6)]
                output = [rad_to_deg(limit(output[i],JOINT_VEL_LIMITS[i])) for i in range(6)]
                
                self._jstmsgs[arm].header.frame_id = ''
                self._jstmsgs[arm].header.stamp = rospy.get_rostime()
                self._jstmsgs[arm].desired.positions=tar
                self._jstmsgs[arm].desired.velocities=ff_rate
                self._jstmsgs[arm].desired.accelerations=ff_acc
                self._jstmsgs[arm].actual.positions=fb
                self._jstmsgs[arm].actual.velocities=self._get_angular_velocity()
                self._jstmsgs[arm].actual.accelerations=ff_acc
                self._jstmsgs[arm].error.positions=error
                self._jstmsgs[arm].error.velocities=[self._jstmsgs[arm].actual.velocities[i] - ff_rate[i] for i in range(6)] 
                self._jstmsgs[arm].error.accelerations=[0.0]*6                 
                self._jstpubs[arm].publish(self._jstmsgs[arm]) 
                self._jstmsgs[arm].header.seq +=1                       
                
                tmp = output[5]
                output = [0.0]*6
                output[5] = tmp
                
                traj = TrajectoryPoint()
                traj.Position.Type = ANGULAR_VELOCITY
                traj.Position.Actuators.Actuator1 = output[0]
                traj.Position.Actuators.Actuator2 = output[1]
                traj.Position.Actuators.Actuator3 = output[2]
                traj.Position.Actuators.Actuator4 = output[3]
                traj.Position.Actuators.Actuator5 = output[4]
                traj.Position.Actuators.Actuator6 = output[5]
                self.SendBasicTrajectory(traj)
               
               
                

                
                
  
                
