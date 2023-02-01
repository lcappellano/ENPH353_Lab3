import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist

def image_callback(image_msg):
    try:
        # Convert the ROS image message to a OpenCV image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # Convert the image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Threshold the image to binary
        ret,thresh = cv2.threshold(gray,127,255,1)
        # Find the contours
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # Find the largest contour
        max_area = 0
        max_contour = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                max_contour = cnt
        # If a valid contour is found
        if max_contour is not None:
            M = cv2.moments(max_contour)
            # Calculate the centroid
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            # Draw a red circle at the centroid
            cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
            # Create a Twist message to control the robot
            twist = Twist()
            # Set the linear and angular velocities based on the centroid position
            twist.linear.x = 0.5
            twist.angular.z = -float(cx - cv_image.shape[0]/2) / (cv_image.shape[0]/2)
            # Publish the Twist message
            cmd_vel_pub.publish(twist)
        # Display the image using OpenCV
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(30)
    except CvBridgeError as e:
        print(e)

def subscribe_to_topics():
    # Initialize the ROS node
    rospy.init_node('subscriber_node', anonymous=True)
    # Create a publisher for cmd_vel
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Subscribe to the camera feed
    rospy.Subscriber('/rrbot/camera1/image_raw', Image, image_callback)

    # Spin to keep the node running
    rospy.spin()

if __name__ == '__main__':
    subscribe_to_topics()
