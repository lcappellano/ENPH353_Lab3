#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def stop_robot():
    # Initialize the node
    print('publisihing')

    rospy.init_node('stop_robot')
    # Create a publisher for the robot's velocity
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # Create a Twist message
    twist = Twist()
    # Set the linear and angular velocities to 0
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

    # Sleep for a short period to ensure the message is sent
    rospy.sleep(1)
    # Publish the Twist message to stop the robot
    pub.publish(twist)
    

if __name__ == '__main__':
    try:
        stop_robot()
    except rospy.ROSInterruptException:
        pass
