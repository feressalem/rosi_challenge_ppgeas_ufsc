#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt! interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. (More on these below)
##
## We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import numpy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from rosi_defy.msg import ManipulatorJoints
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import JointState
from rosi_defy.msg import ManipulatorJoints
from ppgeas.srv import DetectConveyorBelt
from ppgeas.srv import DetectConveyorBeltResponse
from ppgeas.srv import DetectFire
from ppgeas.srv import DetectFireResponse
from geometry_msgs.msg import TwistStamped
from rosi_defy.msg import HokuyoReading
from ppgeas.srv import Touch, TouchResponse
## END_SUB_TUTORIAL

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
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_controller_node',anonymous=True)
    self.seq_points = []
    self.dist = 100
    self.serv = rospy.Service('touch', Touch, self.handle_planning)
    self.sub_manipulatorStates = rospy.Subscriber('/ur5/jointsPositionCurrentState', ManipulatorJoints, self.callback_Mstates)
    self.sub_force = rospy.Subscriber('/ur5/forceTorqueSensorOutput', TwistStamped, self.callback_Force)
    self.sub_hokuyo = rospy.Subscriber('/sensor/hokuyo', HokuyoReading, self.callback_Hokuyo)
    self.joint_states = rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, self.callback_Jstates, queue_size=100)
    self.pub_manipulator = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=100)
    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:

    robot = moveit_commander.RobotCommander()
    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)
    print("INICIO")
    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    #print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    #print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""
    ## END_SUB_TUTORIAL
    # Misc variables
    self.force = 0
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def handle_planning(self,request):
      #Touch.response =
      if request.messages == "levantar":
          self.go_to_pose_goal("levantar")
      elif request.messages == "inicial":
          self.go_to_pose_goal("inicial")
      elif request.messages == "inicial_esquerda":
          self.go_to_pose_goal("inicial_esquerda")
      elif request.messages == "cavalete":
          self.planejamento("cavalete")
      elif request.messages == "cavalete_esquerda":
          self.planejamento("cavalete_esquerda")
      elif request.messages == "cavalete_fogo":
          self.planejamento("cavalete_fogo")
      elif request.messages == "giro1":
          self.go_to_joint_state("giro1")
      elif request.messages == "giro2":
          self.go_to_joint_state("giro2")
      elif request.messages == "tocar":
          self.planejamento("tocar")
      return TouchResponse(success=True, message="Execucao do braco completa")

  # Retorna os estados do manipulador segundo topico do vrep
  def callback_Hokuyo(self, msg):
      #min = 100
      for i in range(0,len(msg.reading),3):
          measure = numpy.array([msg.reading[i],msg.reading[i+1],msg.reading[i+2]])
          norm = numpy.linalg.norm(measure)
          if self.dist > norm:
              self.dist = norm
      #print("Norma: ", min)
      #pos = len(msg.reading)#numpy.round(len(msg.reading)/2)
      #self.laser = msg.reading[pos]
      #print(pos)

  # Retorna os estados do manipulador segundo topico do vrep
  def callback_Force(self, msg):
      self.force = msg.twist.linear.z

  # Retorna os estados do manipulador segundo topico do vrep
  def callback_Mstates(self, msg):
      self.states = msg

  # Retorna as posicoes do manipulador geradas pelo planejamento
  def callback_Jstates(self, msg):
      # pega o item posicao da mensagem e transforma em lista e joga na variavel para publicar
      self.seq_points = []
      self.seq_points.append(list(msg.position))

  def executar(self):
      #print(self.seq_points)
      for juntas in self.seq_points:
          self.pub_manipulator.publish(joint_variable=juntas)
          rospy.sleep(0.2)

  def go_to_joint_state(self,message):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    joint_goal = group.get_current_joint_values()
    if message == "giro1":
        joint_goal[0] = 0.00022912025451660156-pi
    elif message == "giro2":
        joint_goal[0] = 0.00022912025451660156
    #joint_goal[1] = -pi/4
    #joint_goal[2] = 0
    #joint_goal[3] = -pi/2
    #joint_goal[4] = 0
    #joint_goal[5] = pi/3
    #joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_joints = self.group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self,caso):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    if (caso == "tocar"):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = -0.5004
        pose_goal.orientation.y = 0.4992
        pose_goal.orientation.z = 0.5
        pose_goal.orientation.w = 0.5004
        pose_goal.position.x = 0.0
        pose_goal.position.y = 0.0
        pose_goal.position.z = 0.9
        group.set_pose_target(pose_goal)
    elif (caso == "inicial"):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = -0.5004
        pose_goal.orientation.y = 0.4992
        pose_goal.orientation.z = 0.5
        pose_goal.orientation.w = 0.5004
        pose_goal.position.x = -0.10423
        pose_goal.position.y = 0.22235
        pose_goal.position.z = 1.1432
        group.set_pose_target(pose_goal)
    elif (caso == "inicial_esquerda"):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.49911
        pose_goal.orientation.y = 0.50049
        pose_goal.orientation.z = -0.50036
        pose_goal.orientation.w = 0.50004
        pose_goal.position.x = -0.17173
        pose_goal.position.y = -0.35636
        pose_goal.position.z = 1.1432
        group.set_pose_target(pose_goal)
    elif (caso == "levantar"):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = -0.38196
        pose_goal.orientation.y = 0.38082
        pose_goal.orientation.z = 0.59538
        pose_goal.orientation.w = 0.59549
        pose_goal.position.x = -0.10418
        pose_goal.position.y = 0.22239
        pose_goal.position.z = 1.1432
        group.set_pose_target(pose_goal)
    elif (caso == "cavalete"):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = -0.5004
        pose_goal.orientation.y = 0.4992
        pose_goal.orientation.z = 0.5
        pose_goal.orientation.w = 0.5004
        pose_goal.position.x = -0.1040
        pose_goal.position.y = 0.2224
        pose_goal.position.z = 1.0516
        group.set_pose_target(pose_goal)
    elif (caso == "cavalete_fogo"):  #posicao goal x = -6, y = -2
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.5123
        pose_goal.orientation.y = 0.4879
        pose_goal.orientation.z = -0.5110
        pose_goal.orientation.w = 0.4881
        pose_goal.position.x = -0.02088
        pose_goal.position.y = -0.3772
        pose_goal.position.z = 0.9172
        group.set_pose_target(pose_goal)
    elif (caso == "cavalete_esquerda"):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.5123
        pose_goal.orientation.y = 0.4879
        pose_goal.orientation.z = -0.5110
        pose_goal.orientation.w = 0.4881
        pose_goal.position.x = -0.02088
        pose_goal.position.y = -0.3772
        pose_goal.position.z = 0.9172
        group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 10) #0.01

  def plan_cartesian_path(self, scale=1, direcao=0):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through:
    ##
    waypoints = []

    wpose = group.get_current_pose().pose
    #if direcao == 0:
    #    wpose.position.z -= scale*0.02  # First move up (z) #
    #    waypoints.append(copy.deepcopy(wpose))
    if direcao == 0:
        wpose.position.y = wpose.position.y
        wpose.position.z -= scale*0.5
        wpose.position.x = wpose.position.x
        waypoints.append(copy.deepcopy(wpose))
    elif direcao == 1:
        wpose.position.y = wpose.position.y
        wpose.position.z = wpose.position.z
        wpose.position.x += scale*0.02
        waypoints.append(copy.deepcopy(wpose))
    elif direcao == 2:
        wpose.position.x = wpose.position.x
        wpose.position.y = wpose.position.y
        wpose.position.z -= scale*0.02  # First move up (z) #
        waypoints.append(copy.deepcopy(wpose))
    if direcao == 3:
        wpose.position.x = wpose.position.x
        wpose.position.z = wpose.position.z
        wpose.position.y += scale*0.005
        waypoints.append(copy.deepcopy(wpose))

    #wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    #waypoints.append(copy.deepcopy(wpose))

    #wpose.position.y -= scale * 0.1  # Third move sideways (y)
    #waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step 0.01
                                    0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    ## END_SUB_TUTORIAL

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL

  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    group = self.group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
  def center_detection(self):
      rospy.wait_for_service('detect_conveyorbelt')
      try:
          cb_detection = rospy.ServiceProxy('detect_conveyorbelt', DetectConveyorBelt)
          response = cb_detection()
          return response
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

  def detect_fire(self,cx,cy):
      rospy.wait_for_service('detect_fire')
      try:
          cb_detection = rospy.ServiceProxy('detect_fire', DetectFire)
          response = cb_detection()
          return response
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

  def planejamento(self,message):
      if message == "tocar": # lado direito
          window_size_y = 640
          offset_y = -50
          offset_z = -90
          window_size_z = 480
          limit = 10
          response = self.center_detection()
          cy = response.ctdx
          cz = response.ctdy
          i = 0
          self.go_to_pose_goal(caso="cavalete_esquerda")
          while (abs(cy - window_size_y/2 - offset_y) > limit):
              if (cy - window_size_y/2 - offset_y) > limit:
                  cartesian_plan, fraction = self.plan_cartesian_path(scale = -1, direcao = 1)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
              else:
                  cartesian_plan, fraction = self.plan_cartesian_path(scale= 1, direcao = 1)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
              response = self.center_detection()
              cy = response.ctdx
              cz = response.ctdy
          while (abs(cz - window_size_z/2 - offset_z) > limit):
              #print("Executando Z")
              if (cz - window_size_z/2 - offset_z) > limit:
                  cartesian_plan, fraction = self.plan_cartesian_path(scale = 1, direcao = 2)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
              else:
                  cartesian_plan, fraction = self.plan_cartesian_path(scale=-1, direcao = 2)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
              response = self.center_detection()
              cy = response.ctdx
              cz = response.ctdy
          while self.force < 0.4:
              print(self.dist)
              if i < 3:
                  cartesian_plan, fraction = self.plan_cartesian_path(scale = -3, direcao = 3)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  i = i + 1
              else:
                  cartesian_plan, fraction = self.plan_cartesian_path(scale = -1, direcao = 3)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
          self.go_to_pose_goal(caso="cavalete_esquerda")
          self.go_to_pose_goal(caso="inicial_esquerda")

      if message == "cavalete": #lado direito
          window_size_y = 640
          offset_y = -50
          offset_z = -250
          window_size_z = 480
          limit = 10
          response = self.center_detection()
          cy = response.ctdx
          cz = response.ctdy
          i = 0
          #self.go_to_pose_goal(caso="inicial")
          self.go_to_pose_goal(caso="cavalete_esquerda")
          while (abs(cy - window_size_y/2 - offset_y) > limit):
              #print("Executando X")
              if (cy - window_size_y/2 - offset_y) > limit:
                  cartesian_plan, fraction = self.plan_cartesian_path(scale = -1, direcao = 1)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
              else:
                  cartesian_plan, fraction = self.plan_cartesian_path(scale= 1, direcao = 1)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
              response = self.center_detection()
              cy = response.ctdx
              cz = response.ctdy
          while self.force < 0.4:
              if i < 3:
                  #print("ta na norma")
                  cartesian_plan, fraction = self.plan_cartesian_path(scale = -3, direcao = 3)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  i = i + 1
                  #self.executar()
              else:
                  #print("nao ta mais")
                  cartesian_plan, fraction = self.plan_cartesian_path(scale = -1, direcao = 3)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
          self.go_to_pose_goal(caso="cavalete_esquerda")
          self.go_to_pose_goal(caso="inicial_esquerda")
      if message == "cavalete_esquerda":
          window_size_y = 640
          offset_y = -30
          offset_z = -45
          window_size_z = 480
          limit = 10
          response = self.center_detection()
          cy = response.ctdx
          cz = response.ctdy
          #i = 0
          #self.go_to_pose_goal(caso="inicial")
          self.go_to_pose_goal(caso="cavalete_fogo")
          #self.executar()
          #rospy.sleep(5)
          #rospy.sleep(2)
          while (abs(cy - window_size_y/2 - offset_y) > limit):
              #print("Executando X")
              if (cy - window_size_y/2 - offset_y) > limit:
                  cartesian_plan, fraction = self.plan_cartesian_path(scale = 1, direcao = 1)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
              else:
                  cartesian_plan, fraction = self.plan_cartesian_path(scale=-1, direcao = 1)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
              response = self.center_detection()
              cy = response.ctdx
              cz = response.ctdy
          #while (abs(cz - window_size_z/2 - offset_z) > limit):
        #      #print("Executando Z")
        #      if (cz - window_size_z/2 - offset_z) > limit:
        #          cartesian_plan, fraction = self.plan_cartesian_path(scale = 1, direcao = 2)
        #          self.display_trajectory(cartesian_plan)
        #          self.execute_plan(cartesian_plan)
        #          #self.executar()
        #      else:
        #          cartesian_plan, fraction = self.plan_cartesian_path(scale=-1, direcao = 2)
        #          self.display_trajectory(cartesian_plan)
        #          self.execute_plan(cartesian_plan)
        #          #self.executar()
          #response = self.center_detection()
          #cy = response.ctdx
          #cz = response.ctdy
          while self.force < 0.4:
              print(self.dist)
              #print("Executando Y")
              if self.dist > 0.2:
                  #print("ta na norma")
                  cartesian_plan, fraction = self.plan_cartesian_path(scale = 1, direcao = 3)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
              else:
                  #print("nao ta mais")
                  cartesian_plan, fraction = self.plan_cartesian_path(scale=1, direcao = 3)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
                  self.go_to_pose_goal(caso="cavalete")
                  self.go_to_pose_goal(caso="inicial")
      if message == "cavalete_fogo":
          window_size_y = 640
          offset_y = -20
          offset_z = -45
          window_size_z = 480
          limit = 10
          response = self.detect_fire(0,0)
          cy = response.ctdx
          cz = response.ctdy
          i = 0
          self.go_to_pose_goal(caso="cavalete_fogo")
          while (abs(cy - window_size_y/2 - offset_y) > limit):
              #print("Executando X")
              if (cy - window_size_y/2 - offset_y) > limit:
                  cartesian_plan, fraction = self.plan_cartesian_path(scale = -1, direcao = 1)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
              else:
                  cartesian_plan, fraction = self.plan_cartesian_path(scale= 1, direcao = 1)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
              response = self.detect_fire(0,0)
              cy = response.ctdx
              cz = response.ctdy
#          while (abs(cz - window_size_z/2 - offset_z) > limit):
#              #print("Executando Z")
#              if (cz - window_size_z/2 - offset_z) > limit:
#                  cartesian_plan, fraction = self.plan_cartesian_path(scale = 1, direcao = 2)
#                  self.display_trajectory(cartesian_plan)
#                  self.execute_plan(cartesian_plan)
#                  #self.executar()
#              else:
#                  cartesian_plan, fraction = self.plan_cartesian_path(scale=-1, direcao = 2)
#                  self.display_trajectory(cartesian_plan)
#                  self.execute_plan(cartesian_plan)
#                  #self.executar()
#              response = self.detect_fire(0,0)
#              cy = response.ctdx
#              cz = response.ctdy
          while self.force < 0.4:
              #print(self.dist)
              #print("Executando Y")
              if i < 3:
                  #print("ta na norma")
                  cartesian_plan, fraction = self.plan_cartesian_path(scale = -5, direcao = 3)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  i = i + 1
                  #self.executar()
              else:
                  #print("nao ta mais")
                  cartesian_plan, fraction = self.plan_cartesian_path(scale = -1, direcao = 3)
                  self.display_trajectory(cartesian_plan)
                  self.execute_plan(cartesian_plan)
                  #self.executar()
                  #self.go_to_pose_goal(caso="cavalete")
                  self.go_to_pose_goal(caso="cavalete_fogo")
                  self.go_to_pose_goal(caso="inicial_esquerda")

def main():
  try:
    #print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    #raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()
    rospy.spin()
    #print "============ Press `Enter` to plan and display a Cartesian path ..."
    #raw_input()
    #tutorial.go_to_pose_goal()

    #tutorial.planejamento()
    #tutorial.go_to_pose_goal(caso="tocar")
    #tutorial.go_to_pose_goal(caso="inicial")

    #print "============ Press `Enter` to execute a movement using a joint state goal ..."
    #raw_input()
    #tutorial.go_to_joint_state()

    #print "============ Press `Enter` to execute a movement using a pose goal ..."
    #raw_input()
    #tutorial.go_to_pose_goal()

    #print "============ Press `Enter` to plan and display a Cartesian path ..."
    #raw_input()
    #cartesian_plan, fraction = tutorial.plan_cartesian_path()

    #print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
    #raw_input()
    #tutorial.display_trajectory(cartesian_plan)

    #print "============ Press `Enter` to execute a saved path ..."
    #raw_input()
    #tutorial.execute_plan(cartesian_plan)

    #print "============ Press `Enter` to execute a saved path ..."
    #raw_input()
    #tutorial.executar()

    #print "============ Press `Enter` to add a box to the planning scene ..."
    #raw_input()
    #tutorial.add_box()

    #print "============ Press `Enter` to attach a Box to the Panda robot ..."
    #raw_input()
    #tutorial.attach_box()

    #print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    #raw_input()
    #cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    #tutorial.execute_plan(cartesian_plan)

    #print "============ Press `Enter` to detach the box from the Panda robot ..."
    #raw_input()
    #tutorial.detach_box()

    #print "============ Press `Enter` to remove the box from the planning scene ..."
    #raw_input()
    #tutorial.remove_box()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
  #MoveGroupPythonIntefaceTutorial()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/kinetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
