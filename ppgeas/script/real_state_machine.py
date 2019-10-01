#!/usr/bin/env python
import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = [
['one', (2.1, 2.2), (0.0, 0.0, 0.0, 1.0)],
['two', (6.5, 4.43), (0.0, 0.0, -0.984047240305, 0.177907360295)]
]

class Waypoint(State):
    def __init__(self, position, orientation):
        State.__init__(self, outcomes=['success'])
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

    def execute(self, userdata):
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        return 'success'

if __name__ == '__main__':
    rospy.init_node('state_machine')
    state_machine = StateMachine('success')

    with state_machine:
        StateMachine.add('Pose_Inicial', Pose_Inicial(), transitions={'success':'First_Waypoint'})
        StateMachine.add('First_Waypoint', Waypoint(w[1], w[2]), transitions={'success':'Touch_1'})
        StateMachine.add('Medida', Medida(), transitions={'success':waypoint[i]})
        StateMachine.add('Medida', Medida(), transitions={'success':waypoint[i]})
        for i,w in enumerate(waypoints):
            StateMachine.add(w[0], Waypoint(w[1], w[2]),transitions={'success':waypoints[(i + 1) % \ len(waypoints)][0]})

    state_machine.execute()

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
