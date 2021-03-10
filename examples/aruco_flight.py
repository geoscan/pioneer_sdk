from pioneer_sdk import Pioneer
import os
import math
import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
import multiprocessing as mp
from multiprocessing.managers import BaseManager


def image_proc(buff, drone):
    size_of_marker = 0.12  # side length in meters
    # change if calibration_matrix.yaml file is located in not default location
    calibration_file = open(os.path.join(os.getcwd(), '..', "camera_calibration", "result", "calibration_matrix.yaml"))
    parsed_calibration_file = yaml.load(calibration_file, Loader=yaml.FullLoader)
    mtx = np.array(parsed_calibration_file.get("camera_matrix"))
    dist = np.array(parsed_calibration_file.get("dist_coeff"))

    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
    aruco_parameters = aruco.DetectorParameters_create()
    while True:
        try:
            camera_frame = cv2.imdecode(np.frombuffer(drone.get_raw_video_frame(), dtype=np.uint8),
                                        cv2.IMREAD_COLOR)
            gray = cv2.cvtColor(camera_frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_parameters)
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


def drone_control(buff, drone):
    command_x = float(0)
    command_y = float(0)
    command_z = float(1)  # initial flight height
    command_yaw = math.radians(float(0))

    k_p_xy = 0.6
    k_p_z = 0.6
    k_p_yaw = 0.4
    distance = 0.5

    t_vec = None
    r_vec_euler = None

    new_point = True
    new_message = True

    p_r = False

    while True:
        if new_point and new_message:
            drone.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)
            new_point = False
            new_message = False

        if not buff.empty():
            message = buff.get()
            if len(message) == 1 and message[0] == 'end':
                break
            t_vec = message[0]
            r_vec_euler = message[1]
            new_message = True

        if drone.point_reached():
            p_r = True

        if p_r and t_vec is not None and r_vec_euler is not None:
            x_camera = t_vec.item(0)
            y_camera = t_vec.item(1)
            z_camera = t_vec.item(2)
            yaw_camera = math.radians(r_vec_euler.item(1))
            c_yaw_prev = math.cos(command_yaw)
            s_yaw_prev = math.sin(command_yaw)
            command_yaw -= k_p_yaw * yaw_camera
            c_yaw_comm = math.cos(command_yaw)
            s_yaw_comm = math.sin(command_yaw)
            command_x += k_p_xy*(x_camera*c_yaw_prev-z_camera*s_yaw_prev+distance*s_yaw_comm)
            command_y += k_p_xy*(x_camera * s_yaw_prev + z_camera * c_yaw_prev - distance * c_yaw_comm)
            command_z -= k_p_z * y_camera
            new_point = True
            p_r = False


if __name__ == '__main__':
    print('start')
    BaseManager.register('Pioneer', Pioneer)
    manager = BaseManager()
    manager.start()
    pioneer_mini = manager.Pioneer()
    pioneer_mini.arm()
    pioneer_mini.takeoff()

    buffer = mp.Queue(maxsize=1)
    pos_and_orient = mp.Process(target=image_proc, args=(buffer, pioneer_mini))
    drone_flight = mp.Process(target=drone_control, args=(buffer, pioneer_mini))
    pos_and_orient.start()
    drone_flight.start()

    pos_and_orient.join()
    drone_flight.join()

    pioneer_mini.land()
