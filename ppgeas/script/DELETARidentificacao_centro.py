#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import TwistStamped
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from rosi_defy.msg import ManipulatorJoints
from rosi_defy.msg import HokuyoReading
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Image
import cv2, cv_bridge
import imutils

class RosiNodeClass():

    def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)

		if len(approx) > 5:
			shape = "circle"
		#elif len(approx) == 4:
		#	(x, y, w, h) = cv2.boundingRect(approx)
		#	ar = w / float(h)
		#	shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
		#elif len(approx) == 5:
		#	shape = "pentagon"
		#else:
		#	shape =
		return shape

    def callback_Mstates(self, msg):
        self.states = msg.joint_variable[1]

    #def callback_Mforces(self, msg):
    #    self.forces = msg

    def callback_Imu(self, msg):
        self.imu = msg

    def callback_Laser(self, msg):
        self.laser = msg.reading[232]

    def callback_Force(self, msg):
        self.force = msg.twist.linear.z

    def callback_Image(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg) #,desired_encoding='bgr8'
        #resized = cv2.resize(image, None, fx=0.5, fy=0.5)

        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 230, 255, cv2.THRESH_BINARY)[1]
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        # loop over the contours
        for c in cnts:
            M = cv2.moments(c)
            self.cX = int((M["m10"] / M["m00"]))
            self.cY = int((M["m01"] / M["m00"]))
            shape = self.detect(c)
            area = cv2.contourArea(c)
            if shape == "circle" and area > 30:
                cv2.drawContours(self.image, [c], -1, (0, 255, 0), 2)
                cv2.circle(self.image, (self.cX, self.cY), 7, (255, 0, 0), -1)
                #cv2.circle(self.image, (640, 480), 7, (0, 255, 0), -1)
                cv2.putText(self.image, shape, (self.cX, self.cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.imshow("window", self.image)
        cv2.waitKey(10)

    def __init__(self):
        self.force = 0
        self.laser = 0
        self.image = None
        self.cX = 0
        self.cY = 0
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.imu = Imu()
        self.states = ManipulatorJoints()
        self.forces = TwistStamped()
        self.theta = [0,math.pi/15,math.pi/15,-math.pi/7.5,-math.pi/2,0]
        self.orientacao = [0.0,0.0,0.0]
        self.pub_manipulator = rospy.Publisher('/ur5/jointsPosTargetCommand', ManipulatorJoints, queue_size=1)
        self.sub_manipulatorStates = rospy.Subscriber('/ur5/jointsPositionCurrentState', ManipulatorJoints, self.callback_Mstates)
        #self.sub_manipulatorForces = rospy.Subscriber('/ur5/forceTorqueSensorOutput', TwistStamped, self.callback_Mforces)
        self.sub_imu = rospy.Subscriber('/sensor/imu', Imu, self.callback_Imu)
        self.sub_image = rospy.Subscriber('sensor/ur5toolCam', Image, self.callback_Image)
        self.sub_laser = rospy.Subscriber('sensor/hokuyo', HokuyoReading, self.callback_Laser)
        self.sub_force = rospy.Subscriber('ur5/forceTorqueSensorOutput', TwistStamped, self.callback_Force)

        # defining the eternal loop frequency
        node_sleep_rate = rospy.Rate(10)

        # eternal loop (until second order)
        while not rospy.is_shutdown():
            #manipulator = ManipulatorJoints()
            orientation_q = self.imu.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            self.orientacao = [roll,pitch,yaw]
            angulos = []
            distX = 320 - self.cX
            kp = -0.0005
            #for i in range(0, 6):
            #    s = "Escolha o angulos theta" + str(i) + "\n"
            #    ang = float(input(s))
            #    angulos.append(ang) # adding the element

            #print(self.laser)
            #self.theta = [0,0,0,0,0,0]
            #self.theta[0] = kp*distX - math.pi/2 - yaw
            print(self.force)
            if self.force < 0.15:
                self.theta[0] = -math.pi/2 - yaw

                if self.states < 1.2:
                    self.theta[1] += 0.004
                    self.theta[2] -= 0.007
                    self.theta[3] += 0.003
                else:
                    self.theta[2] += 0.003
                    self.theta[3] -= 0.003
            #self.theta[4] = -math.pi/2
            #self.theta[5] = 0

            #angulos = theta
            #size = self.image.shape
            #print(size)
            #print(angulos)
            angulos = self.theta
            self.pub_manipulator.publish(joint_variable=angulos)
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
