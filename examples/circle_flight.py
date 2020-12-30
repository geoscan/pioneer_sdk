from pioneer_sdk import Pioneer
import cv2
import math
import numpy as np

pioneer_mini = Pioneer()
video_frame = bytes()
print('start')
pioneer_mini.arm()
print('armed')
pioneer_mini.takeoff()
print('takeoff')
angle = float(0)
number_of_points = 24
increment = float(360 / number_of_points)
radius = 0.6

command_x = radius * math.cos(math.radians(angle))
command_y = radius * math.sin(math.radians(angle))
command_yaw = angle

if __name__ == '__main__':

    while True:
        print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y,
                                                                              z=float(-1), yaw=command_yaw))
        pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=float(-1), yaw=command_yaw)
        if pioneer_mini.point_reached():
            angle += increment
            command_x = radius * math.cos(math.radians(angle))
            command_y = radius * math.sin(math.radians(angle))
            command_yaw -= math.radians(increment)

        camera_frame = cv2.imdecode(np.frombuffer(pioneer_mini.get_raw_video_frame(), dtype=np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('pioneer_camera_stream', camera_frame)
        key = cv2.waitKey(1)
        if (key == 27) | (angle == 360):  # esc
            print('esc pressed')
            cv2.destroyAllWindows()
            pioneer_mini.land()
            print('landed')
            pioneer_mini.disarm()
            print('disarm')
            exit(0)


