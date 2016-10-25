__author__ = 'Andrew Price'

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class JointConverter:
    def __init__(self, name, reverse, offset):
        self.joint_name = name
        if reverse:
            self.multiplier = -1.0
        else:
            self.multiplier = 1.0
        self.offset = offset

    def convert_angle(self, urdf_angle):
        return (urdf_angle * self.multiplier) + self.offset

    def convert_velocity(self, urdf_velocity):
        return urdf_velocity * self.multiplier


class TrajectoryConverter:
    def __init__(self):
        self.joint_map = {}

    # Default (straight-through) joint
    def add_joint(self, urdf_name):
        self.joint_map[urdf_name] = JointConverter(urdf_name, False, 0)

    def add_joint(self, urdf_name, dynamixel_name, reverse, offset):
        self.joint_map[urdf_name] = JointConverter(dynamixel_name, reverse, offset)

    def convert_trajectory(self, traj):
        new_traj = JointTrajectory()

        new_traj.header = traj.header
        for joint_name in traj.joint_names:
            new_traj.joint_names.append(self.joint_map[joint_name].joint_name)

        for point in traj.points:
            new_point = JointTrajectoryPoint()
            for i, pos in enumerate(point.positions):
                new_point.positions.append(self.joint_map[new_traj.joint_names[i]].convert_angle(pos))
            for i, vel in enumerate(point.velocities):
                new_point.velocities.append(self.joint_map[new_traj.joint_names[i]].convert_velocity(vel))

            new_point.time_from_start = point.time_from_start
            new_traj.points.append(new_point)

        return new_traj