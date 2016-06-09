# This configures the environment variables for a vector rmp V3 simulation
# This is necessary to run before starting the simulation 
#

# If there is an onboard PC powered by the system this will run the watchdog
# to make sure it gets gracefully shutdown before the system power cuts out.
export VECTOR_POWERS_PC_ONBOARD=true

#Default Vector RMP_V3 network 
export VECTOR_IP_ADDRESS=10.66.171.5
export VECTOR_IP_PORT_NUM=8080

# The reference frame for all the accessories and sensors
export VECTOR_PARENT_LINK=base_chassis_link

#Determines if the platform should use 2D or 3D odometry
export VECTOR_USE_2D_ODOMETRY=true

#
# Determines if we should use robot_localization EKF or the onboard odometry;
# set to true for onboard odometry. Platform odometry is fairly accurate and will
# improve in the next few releases, but the robot_localization package offers an alternative
# form of localization that can be used to fuse other forms of odometry sensor data. Such as Visual,
# laser scan matching and landmarking information sources. If this is set to false we use robot
# localization to fuse the wheel odometry, SIC IMU, external IMU, etc.
#
export VECTOR_USE_PLATFORM_ODOMETRY=true

# This will run the full system tele-op node (ie can control all the joints in the system) it is not
# collision aware and is really only meant for demonstration purposes. If set false, teleop just controls
# the platform
export RUN_FULL_SYSTEM_TELEOP=true

# Joystick configurations for joystick set VECTOR_JOY_IS_ATTACHED if the joystick
# is physically attached to this PC
export VECTOR_JOY_IS_ATTACHED=false
export VECTOR_JOY_DEV=/dev/input/js0
export VECTOR_JOY_MAPPING=xbox360
export VECTOR_JOY_DEADZONE=0.1

# Sets whether the costmaps use 3D voxel grid layer (need a 3D sensor enabled)
# export VECTOR_USE_VOXEL_GRID=false support upon request

# External IMU configuration;
export VECTOR_HAS_EXT_IMU=true
export VECTOR_HAS_UM7_IMU=true
export VECTOR_UM7_DEV="/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0-port0"
export EXT_IMU_XYZ="0.15386 0 0.35547"
export EXT_IMU_RPY="0 0 0"

# 2D scanner configuration for launch files; should change hokuyo
# to sick if using a SICK LMS1XX
export VECTOR_HAS_2D_LASER=true
export VECTOR_2D_LASER_NS=hokuyo
export VECTOR_2D_LASER_TOPIC=/hokuyo/scan

# Hokuyo configuration (only supports one by default) watch 
# wavelength on multi-laser systems
export VECTOR_HAS_HOKUYO=true
export VECTOR_HOKUYO_IP=10.66.171.6
export VECTOR_HOKUYO_PORT=10940
export HOKUYO_XYZ="0.35 0 0.2"
export HOKUYO_RPY="3.1415 0 0"
export HOKUYO_MAX_RANGE=10.0
export HOKUYO_MIN_RANGE=0.01

# SICK LIDAR configuration (only supports one by default) watch 
# wavelength on multi-laser systems
export VECTOR_HAS_SICK_TIM=false
export VECTOR_SICK_TIM_IP=10.66.171.8
export VECTOR_SICK_TIM_PORT=2112
export SICK_TIM_XYZ="0.35 0 0.2"
export SICK_TIM_RPY="3.1415 0 0"
export HOKUYO_MAX_RANGE=25.0
export HOKUYO_MIN_RANGE=0.01

# Kinect pan-tilt configuration
export VECTOR_HAS_KINECT_PAN_TILT=true
export KINECT_PAN_TILT_PARENT_LINK=linear_actuator_fixed_link
export KINECT_PAN_TILT_XYZ="0.16688476 0 0.96899800"
export KINECT_PAN_TILT_RPY="0 0 0"
export OCL_IGNORE_SELF_TEST=1

# Machine vision  configuration used for visual odometry, landmark detection, etc
# To add machine vision cabability please contact info@stanleyinnovation.com
export VECTOR_HAS_FLEA3=false
export VECTOR_FLEA3_SN='14472220'
export VECTOR_FLEA3_IS_CALIBRATED=false
export FLEA3_XYZ="0.0518 0 0.35508750"
export FLEA3_RPY="0 0 0"

#Kinova arm configurations (the right arm should be the default if there is only one)
export VECTOR_HAS_KINOVA_ARM=true
export VECTOR_HAS_TWO_KINOVA_ARMS=false
export KINOVA_RIGHT_ARM_SERIAL_NUMBER='PJ00900006153340001' #default arm if there is only one
export KINOVA_LEFT_ARM_SERIAL_NUMBER='' #must specify serial number for 2 arms

#Robotiq gripper configurations
export VECTOR_HAS_ROBOTIQ_GRIPPER=true
export VECTOR_HAS_TWO_ROBOTIQ_GRIPPERS=false
export ROBOTIQ_GRIPPER_COMM_PORT="/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0"
export ROBOTIQ_GRIPPER_BAUDRATE=115200

#Linear actuator
export VECTOR_HAS_LINEAR_ACTUATOR=true


