import rospy
import cv2
import numpy as np
import math

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

# Create a Kalman Filter instance
kf = cv2.KalmanFilter(4, 2)  # 4 states (x, y, dx, dy), 2 measurements (x, y)

def aruco_detection(data, pub):
    global kf

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

        for i in range(0, 4):
            avg_x += corners[0][0][i][0]
            avg_y += corners[0][0][i][1]

        avg_x = int(avg_x / 4)
        avg_y = int(avg_y / 4)

        # Kalman Filter Prediction Step
        prediction = kf.predict()

        # Kalman Filter Update Step with Measurement
        measurement = np.array([[avg_x], [avg_y]], dtype=np.float32)
        kf.correct(measurement)

        # Extract the filtered state from Kalman Filter
        state = kf.statePost

        # Create a Float32MultiArray message
        msg = Float32MultiArray()

        # Set the data in the Float32MultiArray message (filtered position)
        msg.data = [state[0, 0], state[1, 0]]
        pub.publish(msg)  # Publish the message

        # Draw the Kalman filter prediction on the image
        prediction_x, prediction_y = int(prediction[0, 0]), int(prediction[1, 0])
        cv2.circle(cv_image, (prediction_x, prediction_y), 5, (255, 0, 0), -1)
        cv2.putText(cv_image, "Prediction", (prediction_x - 10, prediction_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    else:
        pass

    # Display the annotated frame
    cv2.imshow("Camera feed", cv_image)
    cv2.waitKey(3)


def main():
    # Initialize the ROS node and MoveIt
    print("Aruco detection started")

    # Kalman Filter Initialization
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.processNoiseCov = np.array([[1e-2, 0, 1e-1, 0], [0, 1e-2, 0, 1e-1], [1e-1, 0, 1, 0], [0, 1e-1, 0, 1]], np.float32) * 1e-3
    kf.measurementNoiseCov = np.array([[1e-1, 0], [0, 1e-1]], np.float32) * 1e-1
    kf.errorCovPost = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    rospy.init_node('aruco_detection_node')
    aruco_pose_pub = rospy.Publisher("/aruco/pose", Float32MultiArray, queue_size=10)
    img_sub = rospy.Subscriber("/image_raw", Image, aruco_detection, aruco_pose_pub)
    rospy.spin()

if __name__ == '__main__':
    main()
