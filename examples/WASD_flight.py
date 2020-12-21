from pioneer_sdk import Pioneer
import cv2
import numpy as np

if __name__ == '__main__':
    pioneer_mini = Pioneer()
    video_frame = bytes()
    print('start')
    pioneer_mini.arm()
    print('armed')
    pioneer_mini.takeoff()
    print('takeoff')
    command_x = 0
    command_y = 0
    while True:
        pioneer_mini.get_local_position()
        camera_frame = cv2.imdecode(np.frombuffer(pioneer_mini.get_raw_video_frame(), dtype=np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('pioneer_camera_stream', camera_frame)
        key = cv2.waitKey(1)
        if key == 27:  #esc
            print('esc pressed')
            pioneer_mini.land()
            print('landed')
            pioneer_mini.disarm()
            print('disarm')
            exit(0)
        elif key == ord('w'):
            command_y += 1
            print('w')
            print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y, z=-1, yaw=0))
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=-1, yaw=0)
        elif key == ord('s'):
            command_y -= 1
            print('s')
            print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y, z=-1, yaw=0))
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=-1, yaw=0)
        elif key == ord('a'):
            command_x -= 1
            print('a')
            print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y, z=-1, yaw=0))
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=-1, yaw=0)
        elif key == ord('d'):
            command_x += 1
            print('d')
            print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y, z=-1, yaw=0))
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=-1, yaw=0)

