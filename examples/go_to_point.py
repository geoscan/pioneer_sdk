from pioneer_sdk.asynchronous import Pioneer
import cv2
import math
import numpy as np
import time

if __name__ == '__main__':
    print('start')
    pioneer_mini = Pioneer()
    posX = None
    posY = None
    posZ = None
    pioneer_mini.arm()
    told = time.time()
    while True:
            camera_frame = cv2.imdecode(np.frombuffer(pioneer_mini.get_raw_video_frame(), dtype=np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow('pioneer_camera_stream', camera_frame)
            key = cv2.waitKey(1)

            if key == 27:
                pioneer_mini.land(force=True)

            if time.time() - told > 0.5:
                print(pioneer_mini.command_id)
                told = time.time()

            if pioneer_mini.step_get() == 0:
                pioneer_mini.takeoff()
                if pioneer_mini.in_air():
                    posX, posY, posZ = pioneer_mini.get_local_position(True)
                    print(posX, posY, posZ)
                    pioneer_mini.step_inc()

            elif pioneer_mini.step_get() == 1:
                pioneer_mini.sleep(1, callback=pioneer_mini.step_inc)

            elif pioneer_mini.step_get() == 2:
                pioneer_mini.go_to_local_point(x=1, y=posY, z=posZ+1, yaw=0, callback=pioneer_mini.step_inc)

            elif pioneer_mini.step_get() == 3:
                pioneer_mini.sleep(1, callback=pioneer_mini.step_inc)

            elif pioneer_mini.step_get() == 4:
                pioneer_mini.land()
                if pioneer_mini.landed():
                    pioneer_mini.step_inc()

            elif pioneer_mini.step_get() == 5:
                print("Mission complete!")
                break
