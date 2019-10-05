#!/usr/bin/env python
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

class RosiNodeClass():

    def callback_Mstates(self, msg):
        self.states = msg

    def callback_Mforces(self, msg):
        self.forces = msg

    def callback_Imu(self, msg):
        self.imu = msg

    def __init__(self):
        self.imu = Imu()
        self.states = ManipulatorJoints()
        self.forces = TwistStamped()
        self.theta = [0,0,0,0,0,0]
        self.orientacao = [0.0,0.0,0.0]
        self.pub_manipulator = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=1)
        self.sub_manipulatorStates = rospy.Subscriber('/ur5/jointsPositionCurrentState', ManipulatorJoints, self.callback_Mstates)
        self.sub_manipulatorForces = rospy.Subscriber('/ur5/forceTorqueSensorOutput', TwistStamped, self.callback_Mforces)
        self.sub_imu = rospy.Subscriber('/sensor/imu', Imu, self.callback_Imu)

        # defining the eternal loop frequency
        node_sleep_rate = rospy.Rate(10)

        # eternal loop (until second order)
        while not rospy.is_shutdown():
            #manipulator = ManipulatorJoints()
            orientation_q = self.imu.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            self.orientacao = [roll,pitch,yaw]
            #angulos = []
            #for i in range(0, 6):
            #    s = "Escolha o angulos theta" + str(i) + "\n"
            #    ang = float(input(s))
            #    angulos.append(ang) # adding the element
            angulos = [math.pi-yaw,0,0,0,0,0]
            print(angulos)
            self.theta = angulos
            self.pub_manipulator.publish(joint_variable=self.theta)
            #theta = np.array(self.theta)
            #theta = angulos
            #self.theta = np.array(theta).tolist()

            node_sleep_rate.sleep()

if __name__ == '__main__':
    # initialize the node
    rospy.init_node('rosi_example_node', anonymous=True)

    # instantiate the class
    try:
        node_obj = RosiNodeClass()
    except rospy.ROSInterruptException: pass
