from pioneer_sdk import Pioneer, Camera
import cv2
import math
import numpy as np

angle = float(0)
number_of_points = 10
increment = float(360 / number_of_points)
radius = 0.6
flight_height = float(1)
last_point_reached = False

command_x = radius * math.cos(math.radians(angle))
command_y = radius * math.sin(math.radians(angle))
command_yaw = math.radians(angle)

new_point = True
p_reached = False

if __name__ == '__main__':
    print('start')
    pioneer_mini = Pioneer()
    pioneer_mini.arm()
    pioneer_mini.takeoff()
    camera = Camera()
    while True:
        camera_frame = cv2.imdecode(np.frombuffer(camera.get_frame(), dtype=np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('pioneer_camera_stream', camera_frame)

        if new_point:
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=flight_height, yaw=command_yaw)
            new_point = False

        if pioneer_mini.point_reached():
            p_reached = True

        if p_reached:
            if angle == 360:
                last_point_reached = True
            else:
                angle += increment
                command_x = radius * math.cos(math.radians(angle))
                command_y = radius * math.sin(math.radians(angle))
                command_yaw += math.radians(increment)
                new_point = True
            p_reached = False

        key = cv2.waitKey(1)
        if (key == 27) | last_point_reached:  # esc
            print('esc pressed or mission complete')
            cv2.destroyAllWindows()
            pioneer_mini.land()
            exit(0)
