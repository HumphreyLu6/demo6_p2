#!/usr/bin/env python

import rospy
import smach, smach_ros
import actionlib

from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from kobuki_msgs.msg import Led, BumperEvent

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Search(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['end','found'])
    
    def execute(self, userdata):
        global g_tags, g_target_ARtag_id
        while not rospy.is_shutdown():
            if len(g_tags) != 0:
                g_target_ARtag_id = next(iter(g_tags))
                return 'found'
        return 'end'

class Approach(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['end','arrived', 'approach'])

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
    
    def execute(self, userdata):
        global g_tags, g_target_ARtag_id, g_cmd_vel_pub
        if rospy.is_shutdown():
            return 'end'
        else:
            if g_target_ARtag_id in g_tags:
                tag_pose = g_tags[g_target_ARtag_id]
                if tag_pose.position.x > 0.5:  
                    #move with cmd_vel_pub
                    twist = Twist()
                    twist.linear.x = 0.3
                    g_cmd_vel_pub.publish(twist)
                    return 'approach'
                else:
                    # move with move base
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = str(g_target_ARtag_id)
                    goal.target_pose.pose.position = Point(0, 0, -0.7)
                    goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
                    self.client.send_goal(goal)
                    self.client.wait_for_result()
                    return 'arrived'
            print("lost target")

class Push(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['end','completed'])
    
    def execute(self, userdata):
        if rospy.is_shutdown():
            return 'end'
        self.bumper_sub = rospy.Subscriber(
            'mobile_base/events/bumper', BumperEvent, self.bump_callback)
        self.odom_sub = rospy.Subscriber('/Odom', Odometry, self.odom_callback)

        
    
    def bump_callback(self, msg):
        pass

    def odom_callback(self, msg):
        pass

def ar_callback(msg):
    global g_tags
    for marker in msg.markers:
        g_tags[int(marker.id)] = marker.pose.pose

def main():
    global g_cmd_vel_pub
    rospy.init_node("push_bot")
    g_cmd_vel_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=1)
    ar_tag_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback)

    sm = smach.StateMachine(outcomes = ['end', 'finshed'])
    with sm:

        smach.StateMachine.add(
            'Search', Search(), transitions={'end':'end',
                                             'found':'APPROACH'})
        
        smach.StateMachine.add(
            'APPROACH', Approach(), transitions={'end':'end',
                                                 'arrived':'end',
                                                 'approach':'APPROACH'})

        smach.StateMachine.add(
            'Push', Push(), transitions={'end':'end',
                                         'completed':'end'})   

    sm.execute()
    rospy.spin()

if __name__ == "__main__":
    g_tags = {}
    g_target_ARtag_id = None 
    g_cmd_vel_pub = None
    main()