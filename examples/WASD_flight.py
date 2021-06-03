from pioneer_sdk import Pioneer
import cv2
import math
import numpy as np
import time

command_x = float(0)
command_y = float(0)
command_z = float(1)
command_yaw = math.radians(float(0))
increment_xy = float(0.2)
increment_z = float(0.1)
increment_deg = math.radians(float(90))
new_command = False
old_time = 0

if __name__ == '__main__':
    print('start')
    pioneer_mini = Pioneer()
    pioneer_mini.command_id = 0
    while True:
        camera_frame = cv2.imdecode(np.frombuffer(pioneer_mini.get_raw_video_frame(), dtype=np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('pioneer_camera_stream', camera_frame)
        key = cv2.waitKey(1)
        #print(pioneer_mini.command_id)
        if key == 32:
            pioneer_mini.arm()
            pioneer_mini.command_id = 1
        if key == 27:  # esc
            print('esc pressed')
            #cv2.destroyAllWindows()
            pioneer_mini.command_id = 2
            #break
        if time.time() - old_time > 0.1 and pioneer_mini.command_id == 0:
            if key == ord('w'):
                print('w')
                command_y += increment_xy
                #command_y = 2
                new_command = True
            elif key == ord('s'):
                print('s')
                command_y -= increment_xy
                new_command = True
            elif key == ord('a'):
                print('a')
                command_x -= increment_xy
                new_command = True
            elif key == ord('d'):
                print('d')
                command_x += increment_xy
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
            # else:
            #     command_y = 0

            if new_command:
                pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)
                new_command = False
                old_time = time.time()

        pioneer_mini.command_executer()
