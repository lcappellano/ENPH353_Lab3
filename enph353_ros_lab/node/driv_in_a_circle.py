#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def drive_circle():
    # Initialize the node
    rospy.init_node('drive_circle')
    # Create a publisher for the robot's velocity
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # Set the rate at which to send velocity commands
    rate = rospy.Rate(10)
    # Create a Twist message
    twist = Twist()
    # Set the angular velocity in the z-axis (around the robot's vertical axis)
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = -1
    # Set the linear velocity in the x-axis (forward and backward)
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    # Continuously send velocity commands
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        drive_circle()
    except rospy.ROSInterruptException:
        pass
