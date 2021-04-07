#! /usr/bin/env python

import rospy

import actionlib

from copy import deepcopy
import operator
import time

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
    JointTrajectory
)

from sensor_msgs.msg import JointState
from robotis_controller_msgs.msg import SyncWriteItem

class JointTrajectoryActionServer(object):

    def __init__(self, controller_name):
        self._action_ns = controller_name + '/gripper_joint_trajectory'
        self._as = actionlib.SimpleActionServer(
                self._action_ns,
                FollowJointTrajectoryAction,
                execute_cb=self._on_trajectory_action,
                auto_start = False)
        self._action_name = rospy.get_name()
        self._as.start()
        self._feedback = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()

        # Control rate for gripper commands
        self._control_rate = 50 # Hz

        # Subscribe to gripper joint angle
        # ToDo: I might use the robotis topic instead
        self._sub_joint_state = rospy.Subscriber ('/joint_states', JointState, self._joint_states_cb)
        self._current_gripper_angle = 0

        # Add publisher
        self._pub = rospy.Publisher('/gripper_gazebo_controller/command', JointTrajectory, queue_size=1)
        self._pub_robotis = rospy.Publisher('/robotis/direct/sync_write_item', SyncWriteItem, queue_size=5)
        self._trajectory_command = JointTrajectory()
        self._trajectory_command.joint_names = ["gripper"]
        self._point = JointTrajectoryPoint()
        self._point.time_from_start = rospy.rostime.Duration(1,0)

        # Robotis gripper
        self._goal_position_msg = SyncWriteItem()
        self._goal_position_msg.item_name = "goal_position"
        self._goal_position_msg.joint_name = ["gripper"]
        
        # Cooldwon tolerances
        self.angle_threshold = 0.001 # angle tolerance
        self.time_threshold = 5 # sec

        rospy.loginfo('Successful init')

    def _update_feedback(self, cmd_point, jnt_names, cur_time):
        self._feedback.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        self._feedback.joint_names = jnt_names
        self._feedback.desired = cmd_point
        self._feedback.desired.time_from_start = rospy.Duration.from_sec(cur_time)
        self._feedback.actual.positions = self._get_current_position(jnt_names)
        self._feedback.actual.time_from_start = rospy.Duration.from_sec(cur_time)
        self._feedback.error.positions = map(operator.sub,
                                         self._feedback.desired.positions,
                                         self._feedback.actual.positions
                                        )
        self._feedback.error.time_from_start = rospy.Duration.from_sec(cur_time)
        self._as.publish_feedback(self._feedback)

    def _joint_states_cb(self, msg):
        #print(msg.name)
        for i, n in enumerate(msg.name):
            #print(i, n, self._current_gripper_angle)
            if n == "gripper":
                self._current_gripper_angle = msg.position[i]
                break

    def _get_current_position(self, joint_names):
        return [self._current_gripper_angle]

    def _determine_dimensions(self, trajectory_points):
        # Determine dimensions supplied
        position_flag = True
        velocity_flag = (len(trajectory_points[0].velocities) != 0 and
                         len(trajectory_points[-1].velocities) != 0)
        acceleration_flag = (len(trajectory_points[0].accelerations) != 0 and
                             len(trajectory_points[-1].accelerations) != 0)
        return {'positions':position_flag,
                'velocities':velocity_flag,
                'accelerations':acceleration_flag}

    def _on_trajectory_action(self, goal):
        joint_names = goal.trajectory.joint_names

        if len(joint_names) != 1 or joint_names[0] != "gripper":
            rospy.logerr("%s: Invalid joint angle name: %s, or length: %d" %
                         (self._action_name, str(joint_names), len(joint_names)))
            return

        trajectory_points = goal.trajectory.points

        num_points = len(trajectory_points)
        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (self._action_name,))
            self._as.set_aborted()
            return
        rospy.loginfo("%s: Executing requested joint trajectory" %
                      (self._action_name,))
        rospy.logdebug("Trajectory Points: {0}".format(trajectory_points))
        control_rate = rospy.Rate(self._control_rate)

        dimensions_dict = self._determine_dimensions(trajectory_points)
 
        if num_points == 1:
            # Add current position as trajectory point
            first_trajectory_point = JointTrajectoryPoint()
            first_trajectory_point.positions = self._get_current_position(joint_names)
            # To preserve desired velocities and accelerations, copy them to the first
            # trajectory point if the trajectory is only 1 point.
            if dimensions_dict['velocities']:
                first_trajectory_point.velocities = deepcopy(trajectory_points[0].velocities)
            if dimensions_dict['accelerations']:
                first_trajectory_point.accelerations = deepcopy(trajectory_points[0].accelerations)
            first_trajectory_point.time_from_start = rospy.Duration(0)
            trajectory_points.insert(0, first_trajectory_point)
            num_points = len(trajectory_points)

        # Force Velocites/Accelerations to zero at the final timestep
        # if they exist in the trajectory
        # Remove this behavior if you are stringing together trajectories,
        # and want continuous, non-zero velocities/accelerations between
        # trajectories
        if dimensions_dict['velocities']:
            trajectory_points[-1].velocities = [0.0] * len(joint_names)
        if dimensions_dict['accelerations']:
            trajectory_points[-1].accelerations = [0.0] * len(joint_names)

        # Wait for the specified execution time, if not provided use now
        start_time = goal.trajectory.header.stamp.to_sec()
        if start_time == 0.0:
            start_time = rospy.get_time()

        #print(len(trajectory_points))
        #rospy.loginfo(trajectory_points)
        now_from_start = rospy.get_time() - start_time
        end_time = trajectory_points[-1].time_from_start.to_sec()
        for point in trajectory_points:

            now = rospy.get_time()
            now_from_start = now - start_time

            # Publish gripper angle
            self._trajectory_command.header.stamp = rospy.Time.now()
            self._point = point
            self._trajectory_command.points = [self._point]

            # Publish value for Gazebo joint
            self._pub.publish(self._trajectory_command)

            # Publish value for ROBOTIS gripper
            #print(point.positions)
            self._goal_position_msg.value = [point.positions[0]*643]
            self._pub_robotis.publish(self._goal_position_msg)

            self._update_feedback(deepcopy(point), joint_names, now_from_start)

            control_rate.sleep()

        result = False
        if abs(trajectory_points[-1].positions[0] - self._get_current_position(joint_names)[0]) > self.angle_threshold:
            rospy.loginfo("%s: There is an angle difference: %s and %s, cooldown started!" %
                      (self._action_name, str(trajectory_points[-1].positions[0]), str(self._get_current_position(joint_names)[0])))

            cooldown_start = rospy.get_time()
            while 1:
                now = rospy.get_time()
                if abs(trajectory_points[-1].positions[0] - self._get_current_position(joint_names)[0]) > self.angle_threshold:
                    #rospy.loginfo("%s: There is still an angle difference: %s and %s, elapsed time: %s, waiting..." %
                    #    (self._action_name, str(trajectory_points[-1].positions[0]), str(self._get_current_position(joint_names)[0]), str((now - cooldown_start))))
                    pass
                else:
                    result = True
                    break

                if now - cooldown_start > self.time_threshold:
                    rospy.loginfo("%s: Cooldown expired, aborting." % (self._action_name))
                    break

                control_rate.sleep()

        else:
            rospy.loginfo("%s: Angle difference is OK!" %
                      (self._action_name))
            result = True


        last = trajectory_points[-1]
        now_from_start = rospy.get_time() - start_time
        self._update_feedback(deepcopy(last), joint_names, now_from_start)

        if result == True:
            rospy.loginfo("%s: Joint Trajectory Action Succeeded for gripper" % self._action_name)
        else:
            rospy.loginfo("%s: Joint Trajectory Action Failed for gripper" % self._action_name)

        self._result.error_code = self._result.SUCCESSFUL
        self._as.set_succeeded(self._result)

        


if __name__ == '__main__':
    rospy.init_node('gripper_interface')
    server = JointTrajectoryActionServer('gripper_robotis_controller')
    rospy.spin()