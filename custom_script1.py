#!/usr/bin/env python3

import rospy
from custom_msg.msg import custom # custom file has data type string input
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('send_input')  #this node sends the input to user_input node
    pub = rospy.Publisher('user_input', String, queue_size=10) #Input is the message name that is being sent to the topic 'user_input'

    while not rospy.is_shutdown():
        user_input = input("Enter command (w/s/a/d): ")  #input is used to accept input from the user. This is different from the above 'input' 
        if user_input.lower() in ['w', 's', 'a', 'd']:
            pub.publish(user_input.lower())