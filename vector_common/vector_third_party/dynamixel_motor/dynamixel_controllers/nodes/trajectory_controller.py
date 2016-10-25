#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import sys

import rospy

from dynamixel_driver.dynamixel_serial_proxy import SerialProxy

from dynamixel_controllers.joint_position_controller import JointPositionController
from dynamixel_controllers.joint_position_controller_dual_motor import JointPositionControllerDual
from dynamixel_controllers.joint_trajectory_action_controller import JointTrajectoryActionController

from dynamixel_controllers.joint_trajectory_converter import TrajectoryConverter


class TrajectoryManager:
    def __init__(self, controller_names):
        rospy.init_node('trajectory_controller', anonymous=True)
        rospy.on_shutdown(self.on_shutdown)

        self.controllers = {}
        self.serial_proxies = {}
        self.diagnostics_rate = rospy.get_param('~diagnostics_rate', 1)

        # Load ROS parameters for serial port
        manager_namespace = rospy.get_param('~namespace')
        serial_ports = rospy.get_param('~serial_ports')

        # Set up serial port proxy
        for port_namespace, port_config in serial_ports.items():
            port_name = port_config['port_name']
            baud_rate = port_config['baud_rate']
            min_motor_id = port_config['min_motor_id'] if 'min_motor_id' in port_config else 0
            max_motor_id = port_config['max_motor_id'] if 'max_motor_id' in port_config else 253
            update_rate = port_config['update_rate'] if 'update_rate' in port_config else 5
            error_level_temp = 75
            warn_level_temp = 70

            print(port_name)
            self.serial_proxy = SerialProxy(port_name,
                                            port_namespace,
                                            baud_rate,
                                            min_motor_id,
                                            max_motor_id,
                                            update_rate,
                                            self.diagnostics_rate,
                                            error_level_temp,
                                            warn_level_temp)
            self.serial_proxy.connect()

        self.converter = TrajectoryConverter()
        for c_name in controller_names:
            if "-1" == rospy.get_param(root_path + c_name + "/motor_master", "-1"):
                self.controllers[c_name] = JointPositionController(self.serial_proxy.dxl_io, root_path + c_name, 'arbotix_port')
            else:
                self.controllers[c_name] = JointPositionControllerDual(self.serial_proxy.dxl_io, root_path + c_name, 'arbotix_port')

            dyn_name = rospy.get_param(root_path + c_name + "/joint_name", "")
            reverse = rospy.get_param(root_path + c_name + "/reverse", False)
            offset = rospy.get_param(root_path + c_name + "/offset", 0)

            self.converter.add_joint(dyn_name, dyn_name, reverse, offset)

        for c in self.controllers.values():
            if isinstance(c, JointPositionControllerDual):
                print(c.controller_namespace, "Dual")
            elif isinstance(c, JointPositionController):
                print(c.controller_namespace, "Single")
            else:
                print(type(c))

            c.initialize()

        self.trajectory_manager = JointTrajectoryActionController("trajectory_controller", self.controllers.values())

        self.trajectory_manager.initialize()
        self.trajectory_manager.set_converter(self.converter)
        self.trajectory_manager.start()

        rospy.loginfo("Done intializing")

    def on_shutdown(self):
        self.serial_proxy.disconnect()

if __name__ == '__main__':
    try:
        root_path = rospy.myargv()[1] + "/"
        controller_list = rospy.get_param(root_path)
        for c_name in controller_list:
            print(c_name)
        manager = TrajectoryManager(controller_list)  # Args are names of controllers to employ

        rospy.spin()
    except rospy.ROSInterruptException: pass
