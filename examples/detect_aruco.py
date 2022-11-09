import cv2
from pioneer_sdk import Camera

# Dictionary of aruco-markers
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
# Parameters for marker detection (in this case, default parameters)
arucoParams = cv2.aruco.DetectorParameters_create()

# Connect to the drone camera
camera = Camera()

if __name__ == '__main__':
    while True:
        frame = camera.get_cv_frame()  # Get frame (in this case already decoded)
        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
        # Highlight the decoded markers on the image
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow('video', frame)  # Show an image on the screen
        
        if cv2.waitKey(1) & 0xFF == 27:  # Exit if the ESC key is pressed
            break

    cv2.destroyAllWindows()  # Close all opened openCV windows
