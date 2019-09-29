#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import rospy
import numpy as np
import math
from geometry_msgs.msg import TwistStamped
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):

    def callback_Mstates(self, msg):
            self.states = msg

    def add_box(self, timeout=4):

        #box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0
        box_pose.pose.position.y = 0 # slightly above the end effector
        box_pose.pose.position.z = -.05 # slightly above the end effector
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(2, 2, .01))

        self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        box_name = self.box_name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0
            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def remove_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        scene.remove_world_object(box_name)

        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        self.juntas = []
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        self.states = ManipulatorJoints()

        self.pub_manipulator = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=1)
        self.sub_manipulatorStates = rospy.Subscriber('/ur5/jointsPositionCurrentState', ManipulatorJoints, self.callback_Mstates)

        planning_frame = move_group.get_planning_frame()
        #print "============ Planning frame: %s" % planning_frame

        eef_link = move_group.get_end_effector_link()
        #print "============ End effector link: %s" % eef_link

        group_names = robot.get_group_names()
        #print "============ Available Planning Groups:", robot.get_group_names()

        #print "============ Printing robot state"
        #print robot.get_current_state()
        #print ""

        # Misc variables
        #self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        angulos = self.states.joint_variable
        joint_goal[0] = 0 # = 0 (vrep)
        joint_goal[1] = -math.pi/2 # = 0 (vrep)
        joint_goal[2] = 0 # = 0 (vrep)
        joint_goal[3] = -math.pi/2 # = 0 (vrep)
        joint_goal[4] = 0
        joint_goal[5] = 0

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        current_pose = self.move_group.get_current_pose().pose
        estados = self.robot.get_current_state()
        self.juntas = estados.joint_state.position
        print self.juntas
        #print self.robot.get_current_state()
        return all_close(joint_goal, current_joints, 0.01)

    def executar(self):
        #a = self.juntas[1]
        #self.juntas = [0,0,0,0,0,0]
        #self.juntas[1] = a
        juntas_vrep = list(self.juntas)
        juntas_vrep[1] += math.pi/2
        juntas_vrep[3] += math.pi/2
        print(self.juntas)
        self.pub_manipulator.publish(joint_variable=juntas_vrep)

    def go_to_pose_goal(self):
        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.3
        pose_goal.position.y = 0.5
        pose_goal.position.z = 0.8

        print(move_group.set_pose_target(pose_goal))

        plan = move_group.go(wait=True)


        move_group.stop()

        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        estados = self.robot.get_current_state()
        self.juntas = estados.joint_state.position
        print self.juntas
        #self.pub_manipulator.publish(joint_variable=juntas)
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1
        wpose.position.y += scale * 0.2
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold

        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):
        move_group = self.move_group

        move_group.execute(plan, wait=True)

def main():
    try:
        #print "============ Press `Enter`..."
        #raw_input()
        tutorial = MoveGroupPythonIntefaceTutorial()
        #print "============ Press `Enter`..."
        #raw_input()
        #tutorial.go_to_joint_state()
        print "============ Press `Enter`..."
        raw_input()
        tutorial.add_box()
        print "============ Press `Enter`..."
        raw_input()
        tutorial.go_to_pose_goal()
        print "============ Press `Enter`..."
        raw_input()
        tutorial.executar()
        print "============ Press `Enter`..."
        raw_input()
        tutorial.remove_box()
        #print "============ Press `Enter`..."
        #raw_input()
        #cartesian_plan, fraction = tutorial.plan_cartesian_path()
        #print "============ Press `Enter`..."
        #raw_input()
        #tutorial.display_trajectory(cartesian_plan)
        #print "============ Press `Enter`..."
        #raw_input()
        #tutorial.execute_plan(cartesian_plan)
    except rospy.ROSInterruptException:
      return
    except KeyboardInterrupt:
      return

if __name__ == '__main__':
  main()
