#!/usr/bin/env python
import rospy
import smach
import smach_ros
import actionlib
from geometry_msgs.msg import TwistStamped
from rosi_defy.msg import RosiMovementArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = [
['one', (2.1, 2.2), (0.0, 0.0, 0.0, 1.0)],
['two', (6.5, 4.43), (0.0, 0.0, -0.984047240305, 0.177907360295)]
]

class Waypoint(smach.State):
    def __init__(self, position, orientation):
        smach.State.__init__(self, outcomes=['success'])
        #self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        #self.goal = MoveBaseGoal()
        #self.goal.target_pose.header.frame_id = 'map'
        #self.goal.target_pose.pose.position.x = position[0]
        #self.goal.target_pose.pose.position.y = position[1]
        #self.goal.target_pose.pose.position.z = 0.0
        #self.goal.target_pose.pose.orientation.x = orientation[0]
        #self.goal.target_pose.pose.orientation.y = orientation[1]
        #self.goal.target_pose.pose.orientation.z = orientation[2]
        #self.goal.target_pose.pose.orientation.w = orientation[3]

    def execute(self, userdata):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.pose.position.x = position[0]
        self.goal.target_pose.pose.position.y = position[1]
        self.goal.target_pose.pose.position.z = 0.0
        self.goal.target_pose.pose.orientation.x = orientation[0]
        self.goal.target_pose.pose.orientation.y = orientation[1]
        self.goal.target_pose.pose.orientation.z = orientation[2]
        self.goal.target_pose.pose.orientation.w = orientation[3]
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        return 'success'

class bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['bar_succeeded'])
        #self.force = TwistStamped()
    def execute(self, userdata):
        rospy.sleep(1)
        return 'bar_succeeded'

def arm_cb(ud, msg):
    if msg.movement_array[0].joint_var >= 1:
        return False
    else:
        return True

def main():
    rospy.init_node("monitor_example")
    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('ESTEIRA', smach_ros.MonitorState("/rosi/arms_joints_position", RosiMovementArray, arm_cb), transitions={'invalid':'WAYPOINT_1', 'valid':'ESTEIRA', 'preempted':'ESTEIRA'})
        smach.StateMachine.add('BAR',bar(), transitions={'bar_succeeded':'WAYPOINT_1'})
        smach.StateMachine.add('WAYPOINT_1', Waypoint(waypoints[0][1], waypoints[0][2]), transitions={'success':'BAR'})
        #smach.StateMachine.add('WAYPOINT_2', Waypoint(waypoints[1][1], waypoints[1][2]), transitions={'success':'BAR'})
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    while not rospy.is_shutdown():
        sm.execute()
        rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()
