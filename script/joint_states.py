#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from rosi_defy.msg import ManipulatorJoints

class RosiNodeClass():

    def callback_EstadoManipulador(self, msg):
        self.states = msg

    def __init__(self):
        self.sub = rospy.Subscriber('/ur5/jointsPositionCurrentState', ManipulatorJoints, self.callback_EstadoManipulador)
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.states = ManipulatorJoints()
        self.juntas = JointState()
        self.juntas.header = Header()
        self.juntas.header.stamp = rospy.Time.now()
        self.juntas.name = ['joint_ur5_j1', 'joint_ur5_j2', 'joint_ur5_j3', 'joint_ur5_j4', 'joint_ur5_j5', 'joint_ur5_j6']
        self.juntas.position = []
        self.juntas.velocity = []
        self.juntas.effort = []

        self.rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.juntas.header.stamp = rospy.Time.now()
            self.juntas.position = self.states.joint_variable
            self.pub.publish(self.juntas)
            self.rate.sleep()

if __name__ == '__main__':

	rospy.init_node('joint_state_publisher', anonymous=True)

	try:
		node_obj = RosiNodeClass()
	except rospy.ROSInterruptException: pass
