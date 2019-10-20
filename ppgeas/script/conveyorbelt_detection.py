#!/usr/bin/env python
#
## Codigo comentado
#

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

kernel0 = numpy.ones((10,10), numpy.uint8)
kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(20,20))
#print(kernel2)
#ctrX = 0;
#ctrY = 0;

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/conveyorbelt_test",Image, queue_size=1)
    #self.image_pub2 = rospy.Publisher("/mask",Image,queue_size=10)
    self.serv = rospy.Service('detect_conveyorbelt', DetectConveyorBelt, self.handle_detect_conveyorbelt)
    self.image_pub2 = rospy.Publisher("/image_raw",Image,queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/sensor/ur5toolCam",Image,self.callback)
    self.ctrX = 0
    self.ctrY = 0
    #self.resp = DetectFireResponse()

  def detect(self, c):
      # initialize the shape name and approximate the contour
      shape = "unidentified"
      peri = cv2.arcLength(c, True)
      approx = cv2.approxPolyDP(c, 0.04 * peri, True)
      if len(approx) > 5:
          shape = "circle"
          return shape

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Espelha a imagem3,3
    #cv_image_original = cv_image
    cv_image = cv2.flip(cv_image, +1)
    rangemax = numpy.array([255, 255, 255]) # B, G, R
    rangemin = numpy.array([250, 250, 250])
    # Mascara com a faixa especificada
    thresh = cv2.inRange(cv_image, rangemin, rangemax)
    #gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
    #ret, thresh = cv2.threshold(mask,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # noise removal
    kernel = numpy.ones((3,3),numpy.uint8)
    opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 5)
    erode = cv2.erode(opening, kernel2, iterations=2)
    #opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 35)
    #opening2 = cv2.morphologyEx(opening,cv2.MORPH_OPEN,kernel2, iterations = 2)
    #cv2.imshow("opening2", opening2)
    # sure background area
    sure_bg = cv2.dilate(opening,kernel,iterations=1)
    # Finding sure foreground area
    dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
    ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)
    # Finding unknown region
    sure_fg = numpy.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg,sure_fg)
    edges = cv2.Canny(unknown,100,200)
    #white = numpy.uint8([[[255,255,255]]])
    #hsv_white = cv2.cvtColor(white,cv2.COLOR_BGR2HSV)
    #hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    #lower = numpy.array([0, 100, 100])
    #upper = numpy.array([5, 100, 100])
    # Define a faixa para o filtro BGR
    #rangemax = numpy.array([255, 255, 255]) # B, G, R
    #rangemin = numpy.array([250, 250, 250])
    # Mascara com a faixa especificada
    #mask = cv2.inRange(thresh, rangemin, rangemax)
    #mask = cv2.inRange(cv_image, rangemin, rangemax)
    #masked = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    # Reduz o ruido ao aplicar transformacoes morfologicas
    #close = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    #blackhat = cv2.morphologyEx(mask, cv2.MORPH_BLACKHAT, kernel)
    #erode = mask
    #erode = cv2.morphologyEx(close, cv2.MORPH_OPEN, kernel)
    # Erode uma imagem usando um elemento estruturador
    #erode = cv2.erode(close, kernel2, iterations=3)
    #edges = cv2.Canny(erode,100,200)

    cnts = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    for c in cnts:
        M = cv2.moments(unknown)
        if M["m00"] <> 0:
            cx = int((M["m10"] / M["m00"]))
            cy = int((M["m01"] / M["m00"]))
            shape = self.detect(c)
            area = cv2.contourArea(c)
            #if shape == "circle":
            self.cX = cx
            self.cY = cy
            cv2.drawContours(cv_image, [c], -1, (255, 0, 0), 2)
            cv2.circle(cv_image, (self.cX, self.cY), 7, (255, 0, 0), -1)
            #cv2.putText(cv_image, shape, (self.cX, self.cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    #gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    #erode2 = cv2.erode(gray, kernel2, iterations=2)
    #circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 1,param1=50,param2=30,minRadius=0,maxRadius=0)
    #circles = numpy.uint16(numpy.around(circles))
    #for i in circles[0,:]:
    #    # draw the outer circle
    #    cv2.circle(cv_image,(i[0],i[1]),i[2],(0,255,0),2)
    #    # draw the center of the circle
    #    cv2.circle(cv_image,(i[0],i[1]),2,(0,0,255),3)
    #cv2.imshow("window3", cv_image)

    # Define a regiao de contorno e desenha o perimero e o centroide na imagem
    x, y, w, h = cv2.boundingRect(unknown)
    if h > 5 and w > 5:
      cv2.rectangle(cv_image, (x, y), (x+w, y + h), (0, 0, 255), 3)
      cv2.circle(cv_image, (x+w/2, y+h/2), 5, (0, 0, 255), -1)
      cv2.circle(cv_image, (x+w/2, y), 5, (0, 255, 0), -1)
      self.ctrX = x+w/2
      self.ctrY = y #+h/2
    else:
      self.ctrX = 0
      self.ctrY = 0
    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #  self.image_pub2.publish(self.bridge.cv2_to_imgmsg(mask,"8UC1"))
    #except CvBridgeError as e:
    #  print(e)
    # Apresenta o resultado em uma janela separada
    #cv2.imshow("window", unknown)
    cv2.imshow("window2", cv_image)
    #cv2.imshow("window3", edges)
    cv2.waitKey(10)

  # Metodo para lidar com a requisicao de servico e o retorno da resposta
  def handle_detect_conveyorbelt(self,request):
    return DetectConveyorBeltResponse(ctdx = self.ctrX, ctdy = self.ctrY)

def main(args):
  ic = image_converter()
  # Inicializacao do no
  rospy.init_node('conveyorbelt_detection_server', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
