import cv2
import numpy as np

# ... (Camera matrix and distortion coefficients setup)
distortion_coefficients = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
camera_matrix = np.array([[528.433756558705, 0.0, 320.5],
                          [0.0, 528.433756558705, 240.5],
                          [0.0, 0.0, 1.0]], dtype=np.float64)


# Aruco detection
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

parameters = cv2.aruco.DetectorParameters()
parameters.adaptiveThreshConstant = 10

detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

image = cv2.imread('coordinates.png')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

corners, ids, _ = detector.detectMarkers(gray)

if len(corners) > 0:
    # Draw detected markers
    cv2.aruco.drawDetectedMarkers(image, corners, ids)

    # Estimate pose of the marker(s)
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 1, camera_matrix, distortion_coefficients)

    for i in range(len(rvecs)):
        rvec = rvecs[i][0]
        tvec = tvecs[i][0]

        # Convert rotation vector to rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        # Draw pose axes
        axis_length = 0.3
        endpoints = np.array([
            tvec,
            tvec + rotation_matrix @ np.array([axis_length, 0, 0]),  # X-axis endpoint
            tvec + rotation_matrix @ np.array([0, axis_length, 0]),  # Y-axis endpoint
            tvec + rotation_matrix @ np.array([0, 0, axis_length])   # Z-axis endpoint
        ])
        
        # Project 3D endpoints to image plane
        imgpts, _ = cv2.projectPoints(endpoints, np.zeros(3), np.zeros(3), camera_matrix, distortion_coefficients)
        
        # Draw the pose axes lines
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        for j in range(1, 4):
            pt1 = (int(imgpts[0][0][0]), int(imgpts[0][0][1]))
            pt2 = (int(imgpts[j][0][0]), int(imgpts[j][0][1]))
            img = cv2.line(image, pt1, pt2, colors[j - 1], 3)

# Show the image
cv2.imshow('ArUco Marker Detection', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
