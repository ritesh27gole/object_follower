import rospy
import cv2
import numpy as np
import math

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

'''
---
header: 
  seq: 24
  stamp: 
    secs: 53
    nsecs: 194000000
  frame_id: ''
height: 480
width: 640
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]
K: [528.433756558705, 0.0, 320.5, 0.0, 528.433756558705, 240.5, 0.0, 0.0, 1.0]
R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
P: [528.433756558705, 0.0, 320.5, -0.0, 0.0, 528.433756558705, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
binning_x: 0
binning_y: 0
roi: 
  x_offset: 0
  y_offset: 0
  height: 0
  width: 0
  do_rectify: False
---
'''

def aruco_detection(data,pub):
    avg_x = 0
    avg_y = 0

    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

    parameters = cv2.aruco.DetectorParameters()
    parameters.adaptiveThreshConstant = 10

    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, _ = detector.detectMarkers(cv_image)

    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        for i in range(0,4):
            avg_x += corners[0][0][i][0]
            avg_y += corners[0][0][i][1]

        avg_x = int(avg_x/4)
        avg_y = int(avg_y/4)

        # Create a Float32MultiArray message
        msg = Float32MultiArray()

        # Set the data in the Float32MultiArray message
        msg.data = [avg_x, avg_y]
        pub.publish(msg)  # Publish the message

    else:
        pass

    # Display the annotated frame
    cv2.imshow("Camera feed", cv_image)
    cv2.waitKey(3)

def main():
    # Initialize the ROS node and MoveIt
    print("Aruco detection started")
    rospy.init_node('aruco_detection_node')
    aruco_pose_pub = rospy.Publisher("/aruco/pose", Float32MultiArray, queue_size=10)
    img_sub = rospy.Subscriber("/image_raw", Image, aruco_detection, aruco_pose_pub)
    rospy.spin()

if __name__ == '__main__':
    main()