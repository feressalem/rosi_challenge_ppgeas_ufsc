#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('ppgeas')
import sys
import rospy
import cv2
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ppgeas.srv import DetectConveyorBelt,DetectConveyorBeltResponse
import imutils

kernel = numpy.ones((40,40), numpy.uint8)
kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(40,40))
#print(kernel2)
#ctrX = 0;
#ctrY = 0;

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/fire_test",Image, queue_size=1)
    self.serv = rospy.Service('detect_conveyorbelt', DetectConveyorBelt, self.handle_detect_conveyorbelt)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/sensor/ur5toolCam",Image,self.callback)
    self.ctrX = 0
    self.ctrY = 0
    #self.resp = DetectFireResponse()

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    cv_image = cv2.flip(cv_image, +1)
    rangomax = numpy.array([255, 255, 255]) # B, G, R
    rangomin = numpy.array([250, 250, 250])
    mask = cv2.inRange(cv_image, rangomin, rangomax)
    # reduce the noise
    close = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    erode = cv2.erode(close, kernel2, iterations=2)
    #cv2.imshow("window", erode)

    x, y, w, h = cv2.boundingRect(erode)
    if h > 5 and w > 5:
      cv2.rectangle(cv_image, (x, y), (x+w, y + h), (0, 255, 0), 3)
      cv2.circle(cv_image, (x+w/2, y+h/2), 5, (0, 0, 255), -1)
      self.ctrX = x+w/2
      self.ctrY = y+h/2
    else:
      self.ctrX = 0
      self.ctrY = 0
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
    cv2.imshow("window2", cv_image)
    cv2.waitKey(10)

  def handle_detect_conveyorbelt(self,request):
    return DetectConveyorBeltResponse(ctdx = self.ctrX, ctdy = self.ctrY)

  #def handle_detect_fire(self,req):
    #if self.ctrX > 0 and self.ctrY > 0:
    #  self.fireflag = 1
    #else:
    #  self.fireflag = 0
    #return DetectFireResponse(self.fireflag, self.ctrX, self.ctrY)

def main(args):
  ic = image_converter()
  rospy.init_node('fire_detection_server', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
