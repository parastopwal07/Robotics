#! /usr/bin/env python3 

import rospy
from sensor_msgs.msg import LaserScan   #Laserscan is type of data in /scan rostopic
from geometry_msgs.msg import Twist

dist=0.5

def callback(msg):

    if msg.ranges[300] > dist:   #300 because that laser lies almost in front of the turtlebot

        move.linear.x = 0.5
        move.angular.z = 0.0

    if msg.ranges[300] <=dist:

        rospy.loginfo("Object is detected")
        move.linear.x= 0.0
        move.angular.z = 0.5

    pub.publish(move)

if __name__=='__main__':

    rospy.init_node('controller_node')
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate =rospy.Rate(1)
    move=Twist()

    rospy.spin()
