#!/usr/bin/env python
import sys
import math
import rospy
import imutils
import numpy as np
import cv2, cv_bridge
from sensor_msgs.msg import Image
#from geometry_msgs.msg import TwistStamped
#from rosi_defy.msg import RosiMovement
#from rosi_defy.msg import RosiMovementArray
from ppgeas.srv import DetectConveyorBelt
from ppgeas.srv import DetectConveyorBeltResponse

class CenterDetection():
    def __init__(self):
        self.cX = 0
        self.cY = 0
        self.image = None
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.sub_image = rospy.Subscriber('sensor/ur5toolCam', Image, self.callback_Image)
        self.serv = rospy.Service('detect_conveyorbelt', DetectConveyorBelt, self.handle_detect_conveyorbelt)
        #while not rospy.is_shutdown():
        #    rospy.spin()

    def handle_detect_conveyorbelt(self,request):
        return DetectConveyorBeltResponse(cx = self.cX, cy = self.cY)

    def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"
		peri = cv2.arcLength(c, True)
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)
		if len(approx) > 5:
			shape = "circle"
		return shape

    def callback_Image(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)

        #blur = cv2.medianBlur(self.image,5)
        #gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        #thresh = cv2.threshold(gray, 230, 255, cv2.THRESH_TOZERO)[1]
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 230, 255, cv2.THRESH_BINARY)[1]
        #gray2 = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        #circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 1,param1=50,param2=30,minRadius=50,maxRadius=100)
        #circles = np.uint16(np.around(circles))
        #for i in circles[0,:]:
        #        # draw the outer circle
        #        cv2.circle(self.image,(i[0],i[1]),i[2],(0,255,0),2)
        #        # draw the center of the circle
        #        cv2.circle(self.image,(i[0],i[1]),2,(0,0,255),3)
        #cv2.imshow("window", self.image)

        # loop over the contours
        for c in cnts:
            M = cv2.moments(c)
            if M["m00"] <> 0:
                cx = int((M["m10"] / M["m00"]))
                cy = int((M["m01"] / M["m00"]))
                shape = self.detect(c)
                area = cv2.contourArea(c)
                if shape == "circle" and area > 1000:
                    self.cX = cx
                    self.cY = cy
                    cv2.drawContours(self.image, [c], -1, (0, 255, 0), 2)
                    cv2.circle(self.image, (self.cX, self.cY), 7, (255, 0, 0), -1)
                    cv2.putText(self.image, shape, (self.cX, self.cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.imshow("window", cv2.flip(self.image, 1))

        #cv2.imshow("window", cv2.flip(thresh, 1))
        #print(cv2.getWindowImageRect("window"))
        #cv2.imshow("window2", thresh)
        #cv2.imshow("window3", cv2.flip(self.image, 1))
        cv2.waitKey(10)

def main(args):
    rospy.init_node('conveyorbelt_detection_server', anonymous=True)
    try:
        center_obj = CenterDetection()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
