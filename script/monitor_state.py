#!/usr/bin/env python
import rospy
import smach
import smach_ros
from geometry_msgs.msg import TwistStamped

class bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['bar_succeeded'])
        #self.force = TwistStamped()
    def execute(self, userdata):
        rospy.sleep(3.0)
        return 'bar_succeeded'

def monitor_cb(ud, msg):
    if msg.twist.linear.z < 1:
        return False
    else:
        return True

def main():
    rospy.init_node("monitor_example")
    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('FOO', smach_ros.MonitorState("/ur5/forceTorqueSensorOutput", TwistStamped, monitor_cb), transitions={'invalid':'BAR', 'valid':'FOO', 'preempted':'FOO'})
        smach.StateMachine.add('BAR',bar(), transitions={'bar_succeeded':'FOO'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    main()
