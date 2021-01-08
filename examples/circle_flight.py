from pioneer_sdk import Pioneer
import cv2
import math
import numpy as np

angle = float(0)
number_of_points = 24
increment = float(360 / number_of_points)
radius = 0.6
flight_height = float(-1)
last_point_reached = False

pioneer_mini = Pioneer()
video_frame = bytes()
print('start')
pioneer_mini.arm()
pioneer_mini.takeoff()

command_x = radius * math.cos(math.radians(angle))
command_y = radius * math.sin(math.radians(angle))
command_yaw = math.radians(angle)

if __name__ == '__main__':

    while True:
        camera_frame = cv2.imdecode(np.frombuffer(pioneer_mini.get_raw_video_frame(), dtype=np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('pioneer_camera_stream', camera_frame)

        pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=flight_height, yaw=command_yaw)
        if pioneer_mini.point_reached():
            if angle == 360:
                last_point_reached = True
            else:
                angle += increment
                command_x = radius * math.cos(math.radians(angle))
                command_y = radius * math.sin(math.radians(angle))
                command_yaw -= math.radians(increment)

        key = cv2.waitKey(1)
        if (key == 27) | last_point_reached:  # esc
            print('esc pressed or mission complete')
            cv2.destroyAllWindows()
            pioneer_mini.land()
            pioneer_mini.disarm()
            exit(0)


