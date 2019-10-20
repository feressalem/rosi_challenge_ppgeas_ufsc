#!/usr/bin/env python
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
from geometry_msgs.msg import TwistStamped
from rosi_defy.msg import HokuyoReading
from std_srvs.srv import Trigger, TriggerResponse
import roslib; roslib.load_manifest('ppgeas')
#from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('touch', Trigger, handle_add_two_ints)
    print "Ready to touch."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
