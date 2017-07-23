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
 
 \file   crc16.py

 \brief  This module contains a collection of functions for calculating
         a CRC-16.

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from vector_joint_interface.jaco_joint_controller import SIArmController
from trajectory_smoother import TrajectorySmoother 

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import UInt16

import errno
import math
import rospy
import actionlib
import bisect
import operator
from copy import deepcopy



class VectorArmJTAS(object):
    def __init__(self, arm, serial_number, rate=100.0):
        self._alive = False
        self.init_success = True

        self._action_name = rospy.get_name()
        self._name = arm
        # Action Feedback/Result

        """
        Define the joint names
        """
        self._joint_names = ['%s_shoulder_pan_joint'%arm,
                             '%s_shoulder_lift_joint'%arm,
                             '%s_elbow_joint'%arm,
                             '%s_wrist_1_joint'%arm,
                             '%s_wrist_2_joint'%arm,
                             '%s_wrist_3_joint'%arm]

        """
        Controller parameters from arguments, messages, and dynamic
        reconfigure
        """
        self._trajectory_control_rate = rate  # Hz
        self._goal_time = 0.0
        self._stopped_velocity = 0.0
        self._goal_error = dict()
        self._path_thresh = dict()
        self._traj_smoother = TrajectorySmoother(rospy.get_name(),arm)
        self._ctl = SIArmController(self._joint_names,arm,serial_number)

        if not self._ctl.init_success:
            rospy.logerr("Failed to initialize controller, make sure the serial number exists")
            self.clean_shutdown()
            self.init_success = False
            return

        self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()        
        #self._dyn = reconfig_server
        self._ns = '/vector/%s_arm'%arm
        self._fjt_ns = self._ns + '/follow_joint_trajectory'
        self._server = actionlib.SimpleActionServer(
            self._fjt_ns,
            FollowJointTrajectoryAction,
            execute_cb=self._on_trajectory_action,
            auto_start=False)            
        self._alive = True
        self._server.start()

    def robot_is_enabled(self):
        return True

    def clean_shutdown(self):
        self._ctl.Stop()
        self._alive = False

    def _get_trajectory_parameters(self, joint_names, goal):
        """
        For each input trajectory, if path, goal, or goal_time tolerances
        provided, we will use these as opposed to reading from the
        parameter server/dynamic reconfigure
        """

        """
        Goal time tolerance - time buffer allowing goal constraints to be met
        """
        if goal.goal_time_tolerance:
            self._goal_time = goal.goal_time_tolerance.to_sec()
        else:
            self._goal_time = 1.0
            
        """
        Stopped velocity tolerance - max velocity at end of execution
        """
        self._stopped_velocity = 0.5

        """
        Path execution and goal tolerances per joint
        """
        for jnt in joint_names:
            if jnt not in self._joint_names:
                rospy.logerr(
                    "%s: Trajectory Aborted - Provided Invalid Joint Name %s" %
                    (self._action_name, jnt,))
                self._result.error_code = self._result.INVALID_JOINTS
                self._server.set_aborted(self._result)
                return
                
            """
            Path execution tolerance
            """
            self._path_thresh[jnt] = 0.5
            if goal.path_tolerance:
                for tolerance in goal.path_tolerance:
                    if jnt == tolerance.name:
                        self._path_thresh[jnt] = tolerance.position
            
            """
            Goal error tolerance
            """
            self._goal_error[jnt] = 0.5
            if goal.goal_tolerance:
                for tolerance in goal.goal_tolerance:
                    if jnt == tolerance.name:
                        self._goal_error[jnt] = tolerance.position

    def _get_current_position(self, joint_names):
        return self._ctl.GetCurrentJointPosition(joint_names)

    def _get_current_velocities(self, joint_names):
        return self._ctl.GetCurrentJointVelocity(joint_names)

    def _get_current_errors(self, joint_names):
        error = self._ctl.GetCurrentJointPositionError(joint_names)
        return zip(joint_names, error)       

    def _update_feedback(self, cmd_point, joint_names, cur_time):
        self._fdbk.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        self._fdbk.joint_names = joint_names
        self._fdbk.desired = cmd_point
        self._fdbk.desired.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.actual.positions = self._get_current_position(joint_names)
        self._fdbk.actual.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.error.positions = map(operator.sub,
                                         self._fdbk.desired.positions,
                                         self._fdbk.actual.positions
                                        )
        self._fdbk.error.time_from_start = rospy.Duration.from_sec(cur_time)
        self._server.publish_feedback(self._fdbk)

    def _command_stop(self): 
        self._ctl.SetPositionHold()
        self._ctl.ClearPositionHold()

    def _command_joints(self, joint_names, point, dimensions_dict):
        if self._server.is_preempt_requested() or not self.robot_is_enabled():
            rospy.loginfo("%s: Trajectory Preempted" % (self._action_name,))
            self._server.set_preempted()
            self._command_stop()
            return False

        deltas = self._get_current_errors(joint_names) 
        for delta in deltas:
            if ((math.fabs(delta[1]) >= self._path_thresh[delta[0]]
                and self._path_thresh[delta[0]] >= 0.0)) or not self.robot_is_enabled():
                rospy.logerr("%s: Exceeded Error Threshold on %s: %s" %
                             (self._action_name, delta[0], str(delta[1]),))
                self._result.error_code = self._result.PATH_TOLERANCE_VIOLATED
                self._server.set_aborted(self._result)
                self._command_stop()
                return False
                
        pos = dict(zip(joint_names, point.positions))
        vel = dict(zip(joint_names, [0.0]*len(joint_names)))
        acc = dict(zip(joint_names, [0.0]*len(joint_names)))
        if dimensions_dict['velocities']:
            vel = dict(zip(joint_names, point.velocities))
        if dimensions_dict['accelerations']:
            acc = dict(zip(joint_names, point.accelerations))
        
        if self._alive:
            self._ctl.CommandJoints(pos, vel, acc)
        return True
        
    def _check_goal_state(self, joint_names, last):
        for error in self._get_current_errors(joint_names):
            print error
            if (self._goal_error[error[0]] > 0
                    and self._goal_error[error[0]] < math.fabs(error[1])):
                return error[0]
        if (self._stopped_velocity > 0.0 and
            max([abs(cur_vel) for cur_vel in self._get_current_velocities(joint_names)]) >
                self._stopped_velocity):
            return False
        else:
            return True

    def _on_trajectory_action(self, goal):
        joint_names = goal.trajectory.joint_names
        self._get_trajectory_parameters(joint_names, goal)
        success,results = self._traj_smoother.ProcessTrajectory(goal.trajectory, 
                                                                self._get_current_position(joint_names),
                                                                False)
        if not success:
            self._server.set_aborted()
            return
            
        """
        Copy the results to variables that make sense namewise
        """
        dimensions_dict   = results[0] 
        b_matrix          = results[1]
        trajectory_points = results[2]
        pnt_times         = results[3]
        num_points        = results[4]
        
        """
        Wait for the specified execution time, if not provided use now
        """
        start_time = goal.trajectory.header.stamp.to_sec()
        now = rospy.get_time()
        if start_time == 0.0:
            start_time = rospy.get_time()
        while start_time > now:
            now = rospy.get_time()
        
        """
        Loop until end of trajectory time.  Provide a single time step
        of the control rate past the end to ensure we get to the end.
        Keep track of current indices for spline segment generation
        """
        control_rate = rospy.Rate(self._trajectory_control_rate)
        now_from_start = rospy.get_time() - start_time
        end_time = trajectory_points[-1].time_from_start.to_sec()
        while (now_from_start < end_time and not rospy.is_shutdown() and
               self.robot_is_enabled()):
            now = rospy.get_time()
            now_from_start = now - start_time
            idx = bisect.bisect(pnt_times, now_from_start)

            """
            Calculate percentage of time passed in this interval
            """
            if idx >= num_points:
                cmd_time = now_from_start - pnt_times[-1]
                t = 1.0
            elif idx >= 0:
                cmd_time = (now_from_start - pnt_times[idx-1])
                t = cmd_time / (pnt_times[idx] - pnt_times[idx-1])
            else:
                cmd_time = 0.0
                t = 0.0

            point = self._traj_smoother.GetBezierPoint(b_matrix, 
                                                       idx, 
                                                       t, 
                                                       cmd_time, 
                                                       dimensions_dict)

            """
            Command Joint Position, Velocity, Acceleration
            """
            command_executed = self._command_joints(joint_names, point, dimensions_dict)
            self._update_feedback(deepcopy(point), joint_names, now_from_start)

            """
            Break the loop if the command cannot be executed
            """
            if not command_executed:
                return
            control_rate.sleep()
            
        """
        Keep trying to meet goal until goal_time constraint expired
        """
        last = trajectory_points[-1]
        last_time = trajectory_points[-1].time_from_start.to_sec()
        end_angles = dict(zip(joint_names, last.positions))
        print end_angles
        while (now_from_start < (last_time + self._goal_time)
               and not rospy.is_shutdown() and self.robot_is_enabled()):
            if not self._command_joints(joint_names, last, dimensions_dict):
                return
            now_from_start = rospy.get_time() - start_time
            self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)
            control_rate.sleep()

        now_from_start = rospy.get_time() - start_time
        self._update_feedback(deepcopy(last), joint_names,
                                  now_from_start)

        """
        Verify goal constraint
        """
        result = self._check_goal_state(joint_names, last)
        
        if result is True:
            rospy.loginfo("%s: Joint Trajectory Action Succeeded for %s arm" %
                          (self._action_name, self._name))
            self._result.error_code = self._result.SUCCESSFUL
            self._server.set_succeeded(self._result)
        elif result is False:
            rospy.logerr("%s: Exceeded Max Goal Velocity Threshold for %s arm" %
                         (self._action_name, self._name))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        else:
            rospy.logerr("%s: Exceeded Goal Threshold Error %s for %s arm" %
                         (self._action_name, result, self._name))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        self._command_stop()
