import cv2
import numpy as np
from pioneer_sdk import Camera


def load_coefficients(path):
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    camera_matrix = cv_file.getNode("mtx").mat()
    dist_coeffs = cv_file.getNode("dist").mat()

    cv_file.release()
    return camera_matrix, dist_coeffs


# Dictionary of aruco-markers
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
# Parameters for marker detection (in this case, default parameters)
aruco_params = cv2.aruco.DetectorParameters()
# Create instance of ArucoDetector.
# Required starting from version opencv 4.7.0
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Load camera matrix and distortion coefficients from file
camera_matrix, dist_coeffs = load_coefficients("data.yml")

size_of_marker = 0.05  # side length in meters

# Coordinates of marker corners
points_of_marker = np.array(
    [
        (size_of_marker / 2, -size_of_marker / 2, 0),
        (-size_of_marker / 2, -size_of_marker / 2, 0),
        (-size_of_marker / 2, size_of_marker / 2, 0),
        (size_of_marker / 2, size_of_marker / 2, 0),
    ]
)

# Connect to the drone camera

if __name__ == "__main__":
    camera = Camera()
    while True:
        frame = camera.get_cv_frame()  # Get frame (in this case already decoded)
        if frame is not None:
            # Detect markers
            corners, ids, rejected = aruco_detector.detectMarkers(frame)
            if corners:
                # Calculate pose for first detected marker
                success, rvecs, tvecs = cv2.solvePnP(
                    points_of_marker, corners[0], camera_matrix, dist_coeffs
                )
                print(success, tvecs)
                # Draw axes
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs, tvecs, 0.1)
            cv2.imshow("video", frame)  # Show an image on the screen

        if cv2.waitKey(1) == 27:  # Exit if the ESC key is pressed
            break

    cv2.destroyAllWindows()  # Close all opened openCV windows
