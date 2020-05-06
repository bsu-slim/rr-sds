#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import re

vel_msg = Twist()

def callback(data):
    #Since we are moving just in x-axis

    if re.search("straight",str(data.data), re.IGNORECASE):
        print("you wanted to move straight ahead")
        vel_msg.linear.x = 1
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
    elif re.search("back",str(data.data), re.IGNORECASE):
        print("you wanted to back up")
        vel_msg.linear.x = -1
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        # Starts a new node
        rospy.init_node('TurtleSim_DM', anonymous=True)
        velocity_subscriber = rospy.Subscriber('asr',String, callback=callback)
        velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException: pass