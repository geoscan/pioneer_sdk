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
    command_x = float(0)
    command_y = float(0)
    command_z = float(-1)
    command_yaw = float(0)
    while True:
        pioneer_mini.get_local_position()
        camera_frame = cv2.imdecode(np.frombuffer(pioneer_mini.get_raw_video_frame(), dtype=np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('pioneer_camera_stream', camera_frame)
        key = cv2.waitKey(1)
        if key == 27:  # esc
            print('esc pressed')
            cv2.destroyAllWindows()
            pioneer_mini.land()
            print('landed')
            pioneer_mini.disarm()
            print('disarm')
            exit(0)
        elif key == ord('w'):
            command_y += float(1)
            print('w')
            print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y,
                                                                                  z=command_z, yaw=command_yaw))
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)
        elif key == ord('s'):
            command_y -= float(1)
            print('s')
            print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y,
                                                                                  z=command_z, yaw=command_yaw))
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)
        elif key == ord('a'):
            command_x -= float(1)
            print('a')
            print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y,
                                                                                  z=command_z, yaw=command_yaw))
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)
        elif key == ord('d'):
            command_x += float(1)
            print('d')
            print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y,
                                                                                  z=command_z, yaw=command_yaw))
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)
        elif key == ord('q'):
            command_yaw -= float(90)
            print('d')
            print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y,
                                                                                  z=command_z, yaw=command_yaw))
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)
        elif key == ord('e'):
            command_yaw += float(90)
            print('e')
            print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y,
                                                                                  z=command_z, yaw=command_yaw))
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)
        elif key == ord('h'):
            command_z += float(-0.1)
            print('w')
            print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y,
                                                                                  z=command_z, yaw=command_yaw))
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)
        elif key == ord('l'):
            command_z -= float(-0.1)
            print('w')
            print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y,
                                                                                  z=command_z, yaw=command_yaw))
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)


