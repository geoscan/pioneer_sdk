import cv2
from pioneer_sdk import Camera

# Dictionary of aruco-markers
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
# Parameters for marker detection (in this case, default parameters)
aruco_params = cv2.aruco.DetectorParameters()
# Create instance of ArucoDetector.
# Required starting from version opencv 4.7.0
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# Connect to the drone camera
camera = Camera()

if __name__ == "__main__":
    while True:
        frame = camera.get_cv_frame()  # Get frame (in this case already decoded)
        # Detect markers
        corners, ids, rejected = aruco_detector.detectMarkers(frame)
        # Highlight the decoded markers on the image
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("video", frame)  # Show an image on the screen

        if cv2.waitKey(1) == 27:  # Exit if the ESC key is pressed
            break

    cv2.destroyAllWindows()  # Close all opened openCV windows
