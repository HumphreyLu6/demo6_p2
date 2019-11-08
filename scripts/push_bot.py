#!/usr/bin/env python

import rospy
import smach, smach_ros
import actionlib
import util

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

        # self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # self.client.wait_for_server()
    
    def execute(self, userdata):
        global g_tags, g_target_ARtag_id, g_cmd_vel_pub
        if rospy.is_shutdown():
            return 'end'
        else:
            if g_target_ARtag_id in g_tags:
                tag_pose = g_tags[g_target_ARtag_id]
                if tag_pose.position.x > 0.7:  
                    #move with cmd_vel_pub
                    print(tag_pose.position.x)
                    twist = Twist()
                    twist.linear.x = 0.3
                    g_cmd_vel_pub.publish(twist)
                    return 'approach'
                else:
                    # # move with move base
                    # goal = MoveBaseGoal()
                    # goal.target_pose.header.frame_id = str(g_target_ARtag_id)
                    # goal.target_pose.pose.position = Point(0, 0, -0.7)
                    # goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
                    # self.client.send_goal(goal)
                    # self.client.wait_for_result()
                    util.rotate()
                    util.move(0.5, linear_scale = 0.3)
                    util.rotate(-87)
                    util.move(1.2, linear_scale = 0.3)
                    util.rotate(-87)
                    util.move(0.5, linear_scale = 0.3)
                    util.rotate(-87)
                    return 'arrived'
            print("lost target")
            g_cmd_vel_pub.publish(Twist())
            return 'approach'

class Push(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['end','completed'])
        self.bumper = 1
        self.state = 0
        self.odom = {'x':None, 'y':None}
    
    def execute(self, userdata):
        global g_cmd_vel_pub
        if rospy.is_shutdown():
            return 'end'
        self.bumper_sub = rospy.Subscriber(
            'mobile_base/events/bumper', BumperEvent, self.bump_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        #rospy.wait_for_message('/Odom', Odometry)
        rospy.sleep(1)
        self.pushing = False
        self.stop = False

        ori_x = None
        ori_y = None
        while not self.stop:
            if ori_x == None and ori_y == None and self.odom['x'] != None and self.odom['y'] != None:
                ori_x = self.odom['x']
                ori_y = self.odom['y']
            twist = Twist()
            twist.linear.x = 0.1
            g_cmd_vel_pub.publish(twist)
            if self.bumper == 1:
                if self.state == 1:
                    self.pushing = True
                if self.state == 0:
                    self.pushing = False
            if self.bumper == 0 and self.state == 1:
                g_cmd_vel_pub.publish(Twist())
                util.rotate(5)
            if self.bumper == 2 and self.state == 1:
                g_cmd_vel_pub.publish(Twist())
                util.rotate(-5)
            print(ori_x, ori_y, self.odom['x'], self.odom['y'])
            if ori_x != None and ori_y != None:
                if abs(ori_x - self.odom['x']) > abs(2) or abs(ori_y - self.odom['y']) > abs(2):
                    return 'completed'
            
    def bump_callback(self, msg):
        self.bumper = msg.bumper
        self.state = msg.state

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.odom['x'] = x
        self.odom['y'] = y

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
                                                 'arrived':'PUSH',
                                                 'approach':'APPROACH'})  
        smach.StateMachine.add(
            'PUSH', Push(), transitions={'end':'end',
                                         'completed':'end'}) 
        
        
    sm.execute()
    rospy.spin()

if __name__ == "__main__":
    g_tags = {}
    g_target_ARtag_id = None 
    g_cmd_vel_pub = None
    main()