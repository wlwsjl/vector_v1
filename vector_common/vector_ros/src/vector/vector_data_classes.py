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
 
 \file   vector_data_classes.py

 \brief  a collection of Vector data classes

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from utils import convert_u32_to_float,numToDottedQuad
from vector_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu,MagneticField,JointState
import rospy
import math
import tf
import os

class Vector_Status:
    def __init__(self):
        self._MsgData = Status()
        self._MsgPub = rospy.Publisher('/vector/feedback/status', Status, queue_size=10)
        self._MsgData.header.frame_id = ''
        self._seq = 0
        self.op_mode = 0
        self.init=True
        
    def parse(self,data):

        header_stamp = rospy.get_rostime()
        self.init=False
        self._MsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq
        temp = [data[0],data[1],data[2],data[3]]
        self._MsgData.fault_status_words = temp
        self._MsgData.operational_time = convert_u32_to_float(data[4])
        self._MsgData.operational_state = data[5]
        self.op_mode = data[5]
        self._MsgData.dynamic_response = data[6]
        self._MsgData.machine_id = data[8]
        
        if not rospy.is_shutdown():
            self._MsgPub.publish(self._MsgData)
            self._seq += 1
        
        return header_stamp
    
class Vector_Battery:
    def __init__(self):
        self._MsgData = Battery()
        self._MsgPub = rospy.Publisher('/vector/feedback/battery', Battery, queue_size=10)
        self._MsgData.header.frame_id = ''
        self._seq = 0
        
    def parse(self,data,header_stamp):
        self._MsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq

        self._MsgData.battery_status = data[0]
        self._MsgData.battery_soc = convert_u32_to_float(data[1])
        self._MsgData.battery_voltage_VDC = convert_u32_to_float(data[2])
        self._MsgData.battery_current_A0pk = convert_u32_to_float(data[3])
        self._MsgData.battery_temperature_degC = convert_u32_to_float(data[4])
        
        if not rospy.is_shutdown():
            self._MsgPub.publish(self._MsgData)
            self._seq += 1
        
class Vector_Propulsion:
    def __init__(self):
        self._MsgData = Propulsion()
        self._MsgPub = rospy.Publisher('/vector/feedback/propulsion', Propulsion, queue_size=10)
        self._MsgData.header.frame_id = ''
        self._seq = 0
        
    def parse(self,data,header_stamp):
        self._MsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq
        temp = [data[0],data[1],data[2],data[3]]
        self._MsgData.wheel_motor_status = temp            
        temp = [convert_u32_to_float(data[4]),
                convert_u32_to_float(data[5]),
                convert_u32_to_float(data[6]),
                convert_u32_to_float(data[7])]
        self._MsgData.wheel_motor_current_A0pk = temp

        temp = [convert_u32_to_float(data[8]),
                convert_u32_to_float(data[9]),
                convert_u32_to_float(data[10]),
                convert_u32_to_float(data[11])]
        self._MsgData.wheel_motor_speed_rps = temp

        temp = [convert_u32_to_float(data[12]),
                convert_u32_to_float(data[13]),
                convert_u32_to_float(data[14]),
                convert_u32_to_float(data[15])]
        self._MsgData.wheel_motor_position_rad = temp
        
        self._MsgData.linear_motor_status       = data[16]
        self._MsgData.linear_motor_current_A0pk = convert_u32_to_float(data[17])
        self._MsgData.linear_motor_speed_rps    = convert_u32_to_float(data[18])
        self._MsgData.linear_motor_position_rad = convert_u32_to_float(data[19])
         

        if not rospy.is_shutdown():
            self._MsgPub.publish(self._MsgData)
            self._seq += 1      
        
class Vector_IMU(object):
    def __init__(self):
        self._MsgData = Imu()
        self._MsgPub = rospy.Publisher('/vector/feedback/sic_imu', Imu, queue_size=10)
        self._MsgData.header.frame_id = 'sic_imu_frame'
        self._seq = 0
        ext_imu_topic = rospy.get_param('~ext_imu_topic',None)
        if (None != ext_imu_topic):
            self._ext_imu_sub = rospy.Subscriber(ext_imu_topic,Imu,self.ExternalImuCallback)
            self._ext_imu_pub = rospy.Publisher('/vector/feedback/ext_imu', Imu, queue_size=10)
            self._ext_imu_data = Imu()
    def parse_data(self,data,header_stamp):
        self._MsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq
        self._MsgData.orientation.w = 0
        self._MsgData.orientation.x = 0
        self._MsgData.orientation.y = 0
        self._MsgData.orientation.z = 0
        self._MsgData.orientation_covariance[0] = 99999999.0
        self._MsgData.orientation_covariance[4] = 99999999.0
        self._MsgData.orientation_covariance[8] = 99999999.0
        
        self._MsgData.linear_acceleration.x =  convert_u32_to_float(data[0])
        self._MsgData.linear_acceleration.y =  convert_u32_to_float(data[1]) 
        self._MsgData.linear_acceleration.z =  convert_u32_to_float(data[2])       
        self._MsgData.linear_acceleration_covariance[0] = 0.098 * 0.098
        self._MsgData.linear_acceleration_covariance[4] = 0.098 * 0.098
        self._MsgData.linear_acceleration_covariance[8] = 0.098 * 0.098

        self._MsgData.angular_velocity.x =  convert_u32_to_float(data[3])
        self._MsgData.angular_velocity.y =  convert_u32_to_float(data[4])
        self._MsgData.angular_velocity.z =  convert_u32_to_float(data[5])       
        self._MsgData.angular_velocity_covariance[0] = 0.012 * 0.012
        self._MsgData.angular_velocity_covariance[4] = 0.012 * 0.012
        self._MsgData.angular_velocity_covariance[8] = 0.012 * 0.012
        
        """
        Might add the mag data after some testing indices are 6,7,8
        """
        if not rospy.is_shutdown():
            self._MsgPub.publish(self._MsgData)
            self._seq += 1    
    
    def ExternalImuCallback(self, imu_data):
        self._ext_imu_data.header = imu_data.header
        self._ext_imu_data.orientation = imu_data.orientation
        self._ext_imu_data.orientation_covariance[0] = 0.035 * 0.035
        self._ext_imu_data.orientation_covariance[4] = 0.035 * 0.035
        self._ext_imu_data.orientation_covariance[8] = 0.035 * 0.035
        
        self._ext_imu_data.linear_acceleration = imu_data.linear_acceleration        
        self._ext_imu_data.linear_acceleration_covariance[0] = 0.098 * 0.098
        self._ext_imu_data.linear_acceleration_covariance[4] = 0.098 * 0.098
        self._ext_imu_data.linear_acceleration_covariance[8] = 0.098 * 0.098

        self._ext_imu_data.angular_velocity = imu_data.angular_velocity        
        self._ext_imu_data.angular_velocity_covariance[0] = 0.012 * 0.012
        self._ext_imu_data.angular_velocity_covariance[4] = 0.012 * 0.012
        self._ext_imu_data.angular_velocity_covariance[8] = 0.012 * 0.012
             
        if not rospy.is_shutdown():
            self._ext_imu_pub.publish(self._ext_imu_data)

class Vector_Dynamics:
    def __init__(self):
        self._use_platform_odometry = rospy.get_param('~use_platform_odometry',True)
        self._MsgData = Dynamics()
        self._MsgPub = rospy.Publisher('/vector/feedback/dynamics', Dynamics, queue_size=10)
        self._jointStatePub = rospy.Publisher('/vector/joint_states', JointState, queue_size=10)
        self._jointStateMsg = JointState()
        self._jointStateMsg.name = ['linear_joint']
        self._MsgData.header.frame_id = ''
        self._jointStateMsg.header.frame_id = ''
        
        try:
            self.use_lsm_for_odom = bool(os.environ['VECTOR_USE_LSM_TO_CORRECT_ODOMETRY'])
        except:
            self.use_lsm_for_odom = False

        self._OdomData1 = Odometry()
        self._OdomData2 = Odometry()
        self._OdomPub1 = rospy.Publisher('/vector/feedback/wheel_odometry', Odometry, queue_size=10)
        if (False == self._use_platform_odometry):
            rospy.Subscriber('/vector/odometry/local_filtered', Odometry, self._update_odom_yaw)
        elif (True == self.use_lsm_for_odom):
            self._OdomPub2 = rospy.Publisher('/vector/odometry/local_filtered', Odometry, queue_size=10)
            self.has_recved_lsm = False
            rospy.Subscriber('/vector/lsm/pose', PoseWithCovarianceStamped, self._update_lsm_odom)
            
        else:
            self._OdomPub2 = rospy.Publisher('/vector/odometry/local_filtered', Odometry, queue_size=10)            
        
        
        self._OdomData1.header.frame_id = 'odom'
        self._OdomData1.child_frame_id  = 'base_link'
        
        
        self._OdomData1.pose.covariance = [0.00017,0.0,0.0,0.0,0.0,0.0,
                                          0.0,0.00017,0.0,0.0,0.0,0.0,
                                          0.0,0.0,0.00017,0.0,0.0,0.0,
                                          0.0,0.0,0.0,0.00000,0.0,0.0,
                                          0.0,0.0,0.0,0.0,0.00000,0.0,
                                          0.0,0.0,0.0,0.0,0.0,0.00017]
             
        self._OdomData1.twist.covariance = [0.00017,0.0,0.0,0.0,0.0,0.0,
                                           0.0,0.00017,0.0,0.0,0.0,0.0,
                                           0.0,0.0,0.00017,0.0,0.0,0.0,
                                           0.0,0.0,0.0,0.00000,0.0,0.0,
                                           0.0,0.0,0.0,0.0,0.00000,0.0,
                                           0.0,0.0,0.0,0.0,0.0,0.00017]
                                           
                                           
        self._OdomData2.header.frame_id = self._OdomData1.header.frame_id
        self._OdomData2.child_frame_id = self._OdomData1.child_frame_id
        self._OdomData2.pose.covariance = self._OdomData1.pose.covariance
        self._OdomData2.twist.covariance = self._OdomData1.twist.covariance
        
        self._seq = 0
        
    def _update_lsm_odom(self,msg):
        self._OdomData2.header.stamp = msg.header.stamp
        self._OdomData2.header.seq += 1
        
        (r, p, w) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self._MsgData.odom_yaw_angle_rad = ( w + math.pi) % (2 * math.pi ) - math.pi
        
        rot = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        pos = (msg.pose.pose.position.x,msg.pose.pose.position.y,0.0)
        
        if (False == self.has_recved_lsm):
            self._prev_lsm_update_time = msg.header.stamp.to_sec()
            self._prev_lsm_x = msg.pose.pose.orientation.x
            self._prev_lsm_y = msg.pose.pose.orientation.y
            self._prev_lsm_w = w
            self.has_recved_lsm = True
            dxdt=0.0
            dydt=0.0
            dwdt=0.0 
        else:
            dt = (msg.header.stamp.to_sec() - self._prev_lsm_update_time)
            dxdt= (msg.pose.pose.position.x - self._prev_lsm_x)/dt
            dydt= (msg.pose.pose.position.y - self._prev_lsm_y)/dt
            dwdt= (w - self._prev_lsm_w)/dt
            self._prev_lsm_x = msg.pose.pose.position.x
            self._prev_lsm_y = msg.pose.pose.position.y
            self._prev_lsm_w = w
            self._prev_lsm_update_time = msg.header.stamp.to_sec()
            
            dxdt = (dxdt+self._OdomData1.twist.twist.linear.x)/2.0
            dydt = (dydt+self._OdomData1.twist.twist.linear.y)/2.0
            dwdt = (dwdt+self._OdomData1.twist.twist.angular.z)/2.0
        
        self._OdomData2.twist.twist.linear.x = self._OdomData1.twist.twist.linear.x #dxdt
        self._OdomData2.twist.twist.linear.y = self._OdomData1.twist.twist.linear.y #dydt
        self._OdomData2.twist.twist.linear.z = 0.0
        self._OdomData2.twist.twist.angular.x = 0.0
        self._OdomData2.twist.twist.angular.y = 0.0
        self._OdomData2.twist.twist.angular.z = self._OdomData1.twist.twist.angular.z #dwdt
        self._OdomData2.pose = msg.pose
        
        
        if not rospy.is_shutdown():
            self._OdomPub2.publish(self._OdomData2)        
            br = tf.TransformBroadcaster()
            br.sendTransform(pos,
                             rot,
                             msg.header.stamp,
                             "base_link",
                             "odom") 

    def _update_odom_yaw(self,msg):
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self._MsgData.odom_yaw_angle_rad = ( y + math.pi) % (2 * math.pi ) - math.pi

    def parse(self,data,header_stamp,wheel_circum):
        
        self._MsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq
        
        self._OdomData1.header.stamp = header_stamp
        self._OdomData1.header.seq = self._seq
        
        self._jointStateMsg.header.stamp = header_stamp
        self._jointStateMsg.header.seq = self._seq

        self._MsgData.x_vel_target_mps = convert_u32_to_float(data[0])
        self._MsgData.y_vel_target_mps = convert_u32_to_float(data[1])
        self._MsgData.yaw_rate_target_rps = convert_u32_to_float(data[2])
        self._MsgData.linear_actuator_target_m = convert_u32_to_float(data[3])
        self._MsgData.x_vel_limit_mps = convert_u32_to_float(data[4])
        self._MsgData.y_vel_limit_mps = convert_u32_to_float(data[5])
        self._MsgData.yaw_rate_limit_rps = convert_u32_to_float(data[6])
        self._MsgData.linear_actuator_vel_limit_mps = convert_u32_to_float(data[7])
        
        
        temp = [convert_u32_to_float(data[8]),
                convert_u32_to_float(data[9]),
                convert_u32_to_float(data[10]),
                convert_u32_to_float(data[11])]
        self._MsgData.wheel_vel_mps = temp
        temp = [convert_u32_to_float(data[12]),
                convert_u32_to_float(data[13]),
                convert_u32_to_float(data[14]),
                convert_u32_to_float(data[15])]
        self._MsgData.wheel_pos_m = temp
        
        joint_vel = [convert_u32_to_float(data[16])]
        joint_pos = [convert_u32_to_float(data[17])]
        self._MsgData.linear_actuator_vel_mps = convert_u32_to_float(data[16])
        self._MsgData.linear_actuator_position_m = convert_u32_to_float(data[17])
        
        self._MsgData.x_accel_mps2 = convert_u32_to_float(data[18])
        self._MsgData.y_accel_mps2 = convert_u32_to_float(data[19])
        self._MsgData.yaw_accel_mps2 = convert_u32_to_float(data[20])
        self._OdomData1.twist.twist.linear.x = convert_u32_to_float(data[21])
        self._OdomData1.twist.twist.linear.y = convert_u32_to_float(data[22])
        self._OdomData1.twist.twist.linear.z = 0.0
        self._OdomData1.twist.twist.angular.x = 0.0
        self._OdomData1.twist.twist.angular.y = 0.0
        self._OdomData1.twist.twist.angular.z = convert_u32_to_float(data[23])
        self._OdomData1.pose.pose.position.x = convert_u32_to_float(data[24])
        self._OdomData1.pose.pose.position.y = convert_u32_to_float(data[25])
        self._OdomData1.pose.pose.position.z = 0.0
        
        y = convert_u32_to_float(data[26]) 
        y = ( y + math.pi) % (2 * math.pi ) - math.pi
        self._MsgData.yaw_angle_rad = y
        rot = tf.transformations.quaternion_from_euler(0,0,convert_u32_to_float(data[26]))
        self._OdomData1.pose.pose.orientation.x = rot[0]
        self._OdomData1.pose.pose.orientation.y = rot[1]
        self._OdomData1.pose.pose.orientation.z = rot[2]
        self._OdomData1.pose.pose.orientation.w = rot[3]
        
        x = self._OdomData1.pose.pose.position.x
        y = self._OdomData1.pose.pose.position.y
        z = self._OdomData1.pose.pose.position.z             

        
        self._jointStateMsg.velocity = joint_vel
        self._jointStateMsg.position = joint_pos

        if not rospy.is_shutdown():
            self._OdomPub1.publish(self._OdomData1)
            self._MsgPub.publish(self._MsgData)
            self._jointStatePub.publish(self._jointStateMsg)
            if (True == self._use_platform_odometry) and (False == self.use_lsm_for_odom):
                self._update_odom_yaw(self._OdomData1)
                self._OdomData2 = self._OdomData1
                self._OdomPub2.publish(self._OdomData2)        
                br = tf.TransformBroadcaster()
                br.sendTransform((x,y,z),
                                 rot,
                                 header_stamp,
                                 "base_link",
                                 "odom") 

                self._seq += 1  

class Vector_Configuration:
    def __init__(self):   
        self._MsgData = Configuration()
        self._MsgPub = rospy.Publisher('/vector/feedback/stored_configuration', Configuration, queue_size=10)
        self._MsgData.header.frame_id = ''
        self._MsgData1 = Configuration()
        self._MsgPub1 = rospy.Publisher('/vector/feedback/active_configuration', Configuration, queue_size=10)
        self._MsgData1.header.frame_id = ''
        self._seq = 0 
        self.configuration_feedback = [0]*16

    def SetTeleopConfig(self,data):
        self._MsgData.teleop_x_vel_limit_mps = data[0]
        self._MsgData.teleop_y_vel_limit_mps = data[1]
        self._MsgData.teleop_accel_limit_mps2 = data[2]
        self._MsgData.teleop_yaw_rate_limit_rps = data[3]
        self._MsgData.teleop_yaw_accel_limit_rps2 = data[4] 
        
        self._MsgData1.teleop_x_vel_limit_mps = data[0]
        self._MsgData1.teleop_y_vel_limit_mps = data[1]
        self._MsgData1.teleop_accel_limit_mps2 = data[2]
        self._MsgData1.teleop_yaw_rate_limit_rps = data[3]
        self._MsgData1.teleop_yaw_accel_limit_rps2 = data[4]            

    def parse(self,data,header_stamp):
        self._MsgData.header.stamp = header_stamp
        self._MsgData.header.seq = self._seq 
        self._MsgData1.header.stamp = header_stamp
        self._MsgData1.header.seq = self._seq 
                
        """
        This is the data presently being used by the application
        """
        self._MsgData1.x_vel_limit_mps = convert_u32_to_float(data[0])
        self._MsgData1.y_vel_limit_mps = convert_u32_to_float(data[1])
        self._MsgData1.accel_limit_mps2 = convert_u32_to_float(data[2])
        self._MsgData1.decel_limit_mps2 = convert_u32_to_float(data[3])
        self._MsgData1.dtz_decel_limit_mps2 = convert_u32_to_float(data[4])
        self._MsgData1.yaw_rate_limit_rps = convert_u32_to_float(data[5])
        self._MsgData1.yaw_accel_limit_rps2 = convert_u32_to_float(data[6])
        self._MsgData1.wheel_diameter_m = convert_u32_to_float(data[7])
        self._MsgData1.wheelbase_length_m = convert_u32_to_float(data[8])
        self._MsgData1.wheel_track_width_m = convert_u32_to_float(data[9])
        self._MsgData1.gear_ratio = convert_u32_to_float(data[10])
        self._MsgData1.config_bitmap = data[11]
        self._MsgData1.eth_ip_address = numToDottedQuad(data[12])
        self._MsgData1.eth_port_number = data[13]
        self._MsgData1.eth_subnet_mask = numToDottedQuad(data[14])
        self._MsgData1.eth_gateway = numToDottedQuad(data[15])

        """
        This is the data stored in FRAM
        """
        self.configuration_feedback = data[16:]
        self._MsgData.x_vel_limit_mps = convert_u32_to_float(data[16])
        self._MsgData.y_vel_limit_mps = convert_u32_to_float(data[17])
        self._MsgData.accel_limit_mps2 = convert_u32_to_float(data[18])
        self._MsgData.decel_limit_mps2 = convert_u32_to_float(data[19])
        self._MsgData.dtz_decel_limit_mps2 = convert_u32_to_float(data[20])
        self._MsgData.yaw_rate_limit_rps = convert_u32_to_float(data[21])
        self._MsgData.yaw_accel_limit_rps2 = convert_u32_to_float(data[22])
        self._MsgData.wheel_diameter_m = convert_u32_to_float(data[23])
        self._MsgData.wheelbase_length_m = convert_u32_to_float(data[24])
        self._MsgData.wheel_track_width_m = convert_u32_to_float(data[25])
        self._MsgData.gear_ratio = convert_u32_to_float(data[26])
        self._MsgData.config_bitmap = data[27]
        self._MsgData.eth_ip_address = numToDottedQuad(data[28])
        self._MsgData.eth_port_number = data[29]
        self._MsgData.eth_subnet_mask = numToDottedQuad(data[30])
        self._MsgData.eth_gateway = numToDottedQuad(data[31])

        wheel_circum = self._MsgData1.wheel_diameter_m * math.pi
        
        if not rospy.is_shutdown():
            self._MsgPub.publish(self._MsgData)
            self._MsgPub1.publish(self._MsgData1)
            self._seq += 1
        
        return wheel_circum

class VECTOR_DATA:
    def __init__(self):
        self.status = Vector_Status()
        self.propulsion = Vector_Propulsion()
        self.auxiliary_power = Vector_Battery()
        self.config_param = Vector_Configuration()
        self.dynamics = Vector_Dynamics()
        self.imu = Vector_IMU()


        
    
