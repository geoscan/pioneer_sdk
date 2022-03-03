from pioneer_sdk.asynchronous import Pioneer
import cv2
import math
import numpy as np

command_x = float(0)
command_y = float(0)
command_z = float(1)
command_yaw = math.radians(float(0))
increment_xy = float(0.2)
increment_z = float(0.1)
increment_deg = math.radians(float(20))
new_command = False
old_time = 0

if __name__ == '__main__':
    print('start')
    pioneer_mini = Pioneer()
    while True:
        camera_frame = cv2.imdecode(np.frombuffer(pioneer_mini.get_raw_video_frame(), dtype=np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('pioneer_camera_stream', camera_frame)
        key = cv2.waitKey(1)

        if key == ord('p'):
            command_y, command_x, command_z = pioneer_mini.get_local_position(True)
            print('pos: ', command_x, command_y, command_z)

        if key == 32:
            print('space pressed')
            pioneer_mini.arm()
        if key == ord('t'):
            print('takeoff')
            pioneer_mini.takeoff()
        if key == 27:  # esc
            print('esc pressed')
            pioneer_mini.land(force=True)
        if key == ord('k'):
            print('k')
            if pioneer_mini.landed():
                pioneer_mini.disarm()
        if key == ord('w'):
            print('w')
            command_x += increment_xy
            new_command = True
        elif key == ord('s'):
            print('s')
            command_x -= increment_xy
            new_command = True
        elif key == ord('a'):
            print('a')
            command_y -= increment_xy
            new_command = True
        elif key == ord('d'):
            print('d')
            command_y += increment_xy
            new_command = True
        elif key == ord('q'):
            print('q')
            command_yaw += increment_deg
            new_command = True
        elif key == ord('e'):
            print('e')
            command_yaw -= increment_deg
            new_command = True
        elif key == ord('h'):
            print('h')
            command_z += increment_z
            new_command = True
        elif key == ord('l'):
            print('l')
            command_z -= increment_z
            new_command = True

        if new_command:
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)
            new_command = False
