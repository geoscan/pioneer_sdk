from pioneer_sdk.synchronous import Pioneer
import numpy as np
import cv2

if __name__ == '__main__':
    pioneer_mini = Pioneer()
    while True:
        camera_frame = cv2.imdecode(np.frombuffer(pioneer_mini.get_raw_video_frame(), dtype=np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('pioneer_camera_stream', camera_frame)

        key = cv2.waitKey(1)
        if key == 27:  # esc
            print('esc pressed')
            cv2.destroyAllWindows()
            exit(0)
