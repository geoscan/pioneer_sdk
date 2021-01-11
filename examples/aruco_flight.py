from pioneer_sdk import Pioneer
import os
import time
import math
import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
import multiprocessing as mp


def image_proc(buff):
    size_of_marker = 0.12  # side length in meters
    while True:
        try:
            camera_frame = cv2.imdecode(np.frombuffer(pioneer_mini.get_raw_video_frame(), dtype=np.uint8),
                                        cv2.IMREAD_COLOR)
            gray = cv2.cvtColor(camera_frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)
            if np.all(ids is not None):
                r_vec_rodrigues, t_vec, _ = aruco.estimatePoseSingleMarkers(corners, size_of_marker, mtx, dist)
                aruco.drawDetectedMarkers(camera_frame, corners)
                aruco.drawAxis(camera_frame, mtx, dist, r_vec_rodrigues, t_vec, 0.01)
                aruco.drawDetectedMarkers(camera_frame, corners)
                r_mat = cv2.Rodrigues(r_vec_rodrigues)[0]
                p = np.hstack((r_mat.reshape(3, 3), t_vec.reshape(3, 1)))
                r_vec_euler = cv2.decomposeProjectionMatrix(p)[6]
                if buff.full():
                    buff.get()
                buff.put([t_vec, r_vec_euler])
        except cv2.error:
            continue

        cv2.imshow('marker_detection', camera_frame)
        key = cv2.waitKey(1)

        if key == 27:  # esc
            print('esc pressed')
            cv2.destroyAllWindows()
            if buff.full():
                buff.get()
            buff.put(['end'])
            break


def drone_control(buff):
    command_x = float(0)
    command_y = float(0)
    command_z = float(-1)
    command_yaw = math.radians(float(0))

    k_p_xy = 0.4
    k_p_z = 0.3
    k_p_yaw = 0.7
    distance = 0.5

    t_vec = None
    r_vec_euler = None

    point_send_time = time.time()
    send_timeout = 1  # 1 second
    new_point = True
    new_message = True

    while True:
        if (time.time()-point_send_time) >= send_timeout or (new_point and new_message):
            pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)
            point_send_time = time.time()
            new_point = False
            new_message = False

        if buff.full():
            message = buff.get()
            if len(message) == 1 and message[0] == 'end':
                break
            t_vec = message[0]
            r_vec_euler = message[1]
            new_message = True

        if pioneer_mini.point_reached() and t_vec is not None and r_vec_euler is not None:
            yaw = math.radians(r_vec_euler.item(1))
            command_yaw += k_p_yaw * yaw
            c_command_yaw = math.cos(command_yaw)
            s_command_yaw = math.sin(command_yaw)
            x_camera = t_vec.item(0)
            y_camera = t_vec.item(2)
            z_camera = t_vec.item(1)
            command_x += k_p_xy * (x_camera+distance*s_command_yaw)
            if c_command_yaw >= 0:
                command_y += k_p_xy * (y_camera - distance*c_command_yaw)
            else:
                command_y -= k_p_xy * (y_camera - distance*c_command_yaw)
            command_z += k_p_z * z_camera
            new_point = True


# change if calibration_matrix.yaml file is located in not default location
calibration_file = open(os.path.join(os.getcwd(), '..', "camera_calibration", "result", "calibration_matrix.yaml"))
parsed_calibration_file = yaml.load(calibration_file, Loader=yaml.FullLoader)
mtx = np.array(parsed_calibration_file.get("camera_matrix"))
dist = np.array(parsed_calibration_file.get("dist_coeff"))

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
arucoParameters = aruco.DetectorParameters_create()

print('start')
pioneer_mini = Pioneer()
pioneer_mini.arm()
pioneer_mini.takeoff()

if __name__ == '__main__':

    buffer = mp.Queue(maxsize=1)
    pos_and_orient = mp.Process(target=image_proc, args=(buffer,))
    drone_flight = mp.Process(target=drone_control, args=(buffer,))
    pos_and_orient.start()
    drone_flight.start()

    pos_and_orient.join()
    drone_flight.join()

    pioneer_mini.land()
    pioneer_mini.disarm()
