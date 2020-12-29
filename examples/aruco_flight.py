from pioneer_sdk import Pioneer
import os
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
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParameters)
            if np.all(ids is not None):
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, size_of_marker, mtx, dist)
                # print('tvec:', tvec)
                # print('rvec:', rvec)
                aruco.drawDetectedMarkers(camera_frame, corners)
                aruco.drawAxis(camera_frame, mtx, dist, rvec, tvec, 0.01)
                aruco.drawDetectedMarkers(camera_frame, corners)
                if buff.full():
                    buff.get()
                buff.put([tvec, rvec])
        except cv2.error as e:
            continue

        cv2.imshow('marker_detection', camera_frame)
        key = cv2.waitKey(1)

        if key == 27:  # esc
            print('esc pressed')
            cv2.destroyAllWindows()
            pioneer_mini.land()
            print('landed')
            break


def drone_control(buff):
    command_x = float(0)
    command_y = float(0)
    command_z = float(-1)
    command_yaw = float(0)
    K_p_xy = 0.6
    K_p_z = 0.4
    distance = 0.4
    accuracy = 0.001
    landing_flag = False

    while True:
        if buff.full():
            tvec, rvec = buff.get()
            command_x += K_p_xy * tvec.item(0)
            command_y -= distance - K_p_xy * tvec.item(2)
            command_z += K_p_z * tvec.item(1)

            while not pioneer_mini.point_reached(blocking=True):
                print('sending local point X: {x}, Y: {y}, Z: {z}, YAW: {yaw}'.format(x=command_x, y=command_y,
                                                                                      z=command_z, yaw=command_yaw))
                pioneer_mini.go_to_local_point(x=command_x, y=command_y, z=command_z, yaw=command_yaw)


pioneer_mini = Pioneer()

# change if calibration_matrix.yaml file is located in not default location
calibration_file = open(os.path.join(os.getcwd(), '..', "camera_calibration", "result", "calibration_matrix.yaml"))
parsed_calibration_file = yaml.load(calibration_file, Loader=yaml.FullLoader)
mtx = np.array(parsed_calibration_file.get("camera_matrix"))
dist = np.array(parsed_calibration_file.get("dist_coeff"))

aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
arucoParameters = aruco.DetectorParameters_create()

if __name__ == '__main__':

    print('start')
    pioneer_mini.arm()
    print('armed')
    pioneer_mini.takeoff()
    print('takeoff')

    buffer = mp.Queue(maxsize=1)
    pos_and_orient = mp.Process(target=image_proc, args=(buffer,))
    drone_flight = mp.Process(target=drone_control, args=(buffer,))
    pos_and_orient.start()
    drone_flight.start()

    pos_and_orient.join()
    drone_flight.join()
