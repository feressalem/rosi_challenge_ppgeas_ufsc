#!/usr/bin/env python

import sys
import copy
import rospy
import math
import numpy as np
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from rosi_defy.msg import RosiMovement
from geometry_msgs.msg import TwistStamped
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints
from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ppgeas.srv import Planning
from ppgeas.srv import PlanningResponse
from beginner_tutorials.srv import DetectConveyorBelt
from beginner_tutorials.srv import DetectConveyorBeltResponse

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

class MoveGroupPythonInteface(object):

    # Retorna os estados do manipulador segundo topico do vrep
    def callback_Mstates(self, msg):
            self.states = msg

    # Retorna as posicoes do manipulador geradas pelo planejamento
    def callback_Jstates(self, msg):
            # pega o item posicao da mensagem e transforma em lista e joga na variavel para publicar
            self.seq_points.append(list(msg.position))

    # Adiciona box de restricao para o planejamento nao atravessar o chao
    def add_box(self, timeout=4):
        scene = self.scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0
        box_pose.pose.position.y = 0
        box_pose.pose.position.z = -.05
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

    # Remove box de restricao para o planejamento ao final da execucao
    def remove_box(self, timeout=4):
        box_name = self.box_name
        scene = self.scene
        scene.remove_world_object(box_name)

        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def handle_planning(self,request):
        arm.go_centro(request.cx,request.cy)
        arm.executar()
        return PlanningResponse(self.seq_points)

    # Metodo inicial
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()
        self.pontos=[]
        self.juntas = []
        self.seq_points = []
        self.trajetoria = JointState()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)
        self.serv = rospy.Service('planning', Planning, self.handle_planning)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
        self.states = ManipulatorJoints()
        # Publisher das posicoes da trajetoria
        self.pub_manipulator = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=1)
        # Subscriber dos estados das juntas
        self.sub_manipulatorStates = rospy.Subscriber('/ur5/jointsPositionCurrentState', ManipulatorJoints, self.callback_Mstates)
        # Subscriber dos estados das juntas planejado
        self.joint_states = rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, self.callback_Jstates, queue_size=100)
        # Obtem o frame de referencia
        planning_frame = move_group.get_planning_frame()
        #print "============ Planning frame: %s" % planning_frame
        # Obtem o link do efetuador
        eef_link = move_group.get_end_effector_link()
        #print "============ End effector link: %s" % eef_link
        # Obtem o nome do grupo definido no moveit config
        group_names = robot.get_group_names()
        #print "============ Available Planning Groups:", robot.get_group_names()

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    # Metodo para enviar posicao desejada das juntas
    def go_to_joint_state(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        angulos = self.states.joint_variable
        joint_goal[0] = 0.3 # = 0 (vrep)
        joint_goal[1] = -math.pi/2 # = 0 (vrep)
        joint_goal[2] = 0.5 # = 0 (vrep)
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
        return all_close(joint_goal, current_joints, 0.01)

    # Metodo para enviar sequencia de pontos do planejamento para controle das juntas
    def executar(self):
        for juntas in self.seq_points:
            self.pub_manipulator.publish(joint_variable=juntas)
            rospy.sleep(0.2)

    # Metodo para enviar posicao no espaco desejada
    def go_to_pose_goal(self):
        move_group = self.move_group
        #Posicao Inicial
        #pose_goal.orientation.x = -0.5004
        #pose_goal.orientation.y = 0.4992
        #pose_goal.orientation.z = 0.5
        #pose_goal.orientation.w = 0.5004
        #pose_goal.position.x = -0.10423
        #pose_goal.position.y = 0.22235
        #pose_goal.position.z = 1.1432
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = -0.5004
        pose_goal.orientation.y = 0.4992
        pose_goal.orientation.z = 0.5
        pose_goal.orientation.w = 0.5004
        pose_goal.position.x = 0.0
        pose_goal.position.y = 0.22235
        pose_goal.position.z = 1.0

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()
        current_pose = self.move_group.get_current_pose().pose
        estados = self.robot.get_current_state()
        estado_inicial = self.states.joint_variable
        estado_final = estados.joint_state.position[5:11]
        return all_close(pose_goal, current_pose, 0.01)

    def go_centro(self,cy,cz):
        window_size_y = 320
        window_size_z = 640
        limit = 5
        #while (abs(cy - window_size_y) < limit) and (abs(cz - window_size_z) < limit):
        move_group = self.move_group
        current_pose = self.move_group.get_current_pose().pose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = -0.5004
        pose_goal.orientation.y = 0.4992
        pose_goal.orientation.z = 0.5
        pose_goal.orientation.w = 0.5004
        pose_goal.position.x = 0.0
        pose_goal.position.y = self.move_group.get_current_pose().pose.position.y + (cy - window_size_y/2)/100
        pose_goal.position.z = self.move_group.get_current_pose().pose.position.z + (cz - window_size_z/2)/100

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        return all_close(pose_goal, current_pose, 0.01)

    # Metodo para planejamento de pontos no plano cartesiano
    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.x -= scale * 0.1
        #wpose.position.y += scale * 0.2
        waypoints.append(copy.deepcopy(wpose))

        #wpose.position.x += scale * 0.1
        #wpose.position.z -= scale * 0.1
        #waypoints.append(copy.deepcopy(wpose))

        #wpose.position.y -= scale * 0.1
        #wpose.position.z -= scale * 0.1
        #waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold

        #self.plan = plan
        #print(plan) #plan.joint_trajectory.points
        #print(plan.joint_trajectory.points[0].positions)
        #print(plan.joint_trajectory.points[1].positions)
        #print(plan.joint_trajectory.points[2].positions)
        #print(fraction)
        return plan, fraction

    # Metodo para enviar o planejamento para o rviz
    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        display_trajectory_publisher.publish(display_trajectory);

    # Metodo para executar o planejamento no gazebo
    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

def main():
    try:
        arm = MoveGroupPythonInteface()
        print("============ Press `Enter`...")
        raw_input()
        arm.go_to_pose_goal()
        print("============ Press `Enter`...")
        raw_input()
        arm.executar()
    except rospy.ROSInterruptException:
      return
    except KeyboardInterrupt:
      return

if __name__ == '__main__':
  main()
