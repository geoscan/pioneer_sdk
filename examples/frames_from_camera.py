import cv2
import numpy as np
from pioneer_sdk import Camera

# Connect to the drone camera
camera = Camera()

if __name__ == '__main__':
    while True:
        raw_frame = camera.get_frame()  # Get raw data
        if raw_frame is not None:
            # Decode data to get image
            frame = cv2.imdecode(np.frombuffer(raw_frame, dtype=np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('video', frame)  # Show an image on the screen
        
        if cv2.waitKey(1) == 27:  # Exit if the ESC key is pressed
            break

    cv2.destroyAllWindows()  # Close all opened openCV windows
