import cv2
from pioneer_sdk import Pioneer, Camera
import numpy as np


def load_coefficients(path):
    """Loads camera matrix and distortion coefficients."""
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("mtx").mat()
    dist_matrix = cv_file.getNode("dist").mat()

    cv_file.release()
    return camera_matrix, dist_matrix


if __name__ == "__main__":
    camera_mtx, camera_dist = load_coefficients("data.yml")
    size_of_marker = 0.1  # side length in meters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
    aruco_parameters = cv2.aruco.DetectorParameters_create()
    camera = Camera()
    mini = Pioneer()
    # Go to start point
    mini.arm()
    mini.takeoff()
    mini.go_to_local_point(x=0, y=0, z=1.5, yaw=0)
    while not mini.point_reached():
        pass
    send_manual_speed = False
    while True:
        coordinates = None
        try:
            frame = camera.get_cv_frame()
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
                frame, aruco_dict, parameters=aruco_parameters
            )
            # If markers are detected
            if np.all(ids is not None):
                # Find center of aruco marker on screen
                x_center = int(
                    (
                        corners[0][0][0][0]
                        + corners[0][0][1][0]
                        + corners[0][0][2][0]
                        + corners[0][0][3][0]
                    )
                    // 4
                )
                y_center = int(
                    (
                        corners[0][0][0][1]
                        + corners[0][0][1][1]
                        + corners[0][0][2][1]
                        + corners[0][0][3][1]
                    )
                    // 4
                )
                dot_size = 5
                # Draw red square in center of aruco marker
                frame[
                    y_center - dot_size : y_center + dot_size,
                    x_center - dot_size : x_center + dot_size,
                ] = [0, 0, 255]
                # Draw markers
                cv2.aruco.drawDetectedMarkers(frame, corners)

                r_vec_rodrigues, t_vec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, size_of_marker, camera_mtx, camera_dist
                )
                coordinates = [t_vec.item(0), t_vec.item(1), t_vec.item(2)]
                # If you need to, you can draw the axes using the following code:
                # cv2.drawFrameAxes(frame, camera_mtx, camera_dist, r_vec_rodrigues, t_vec, 0.01)

        except cv2.error:
            continue

        if coordinates is not None:
            # calculate the distance to the marker
            distance = np.sqrt(
                coordinates[0] ** 2 + coordinates[1] ** 2 + coordinates[2] ** 2
            )
            print(f"Distance: {distance}")
            v_y = 0
            yaw_rate = 0
            # set a non-zero speed if the distance is >1.2 m or <0.5 m
            if distance > 1.2:
                v_y = 0.4
            elif distance < 0.5:
                v_y = -0.4
            # set a non-zero rotation speed if the center of the marker is on the side of the screen
            if x_center < frame.shape[0] / 3:
                yaw_rate = -0.4
            elif x_center > frame.shape[0] * 2 / 3:
                yaw_rate = 0.4
            # if at least one of the values of `v_y` or `yaw_rate` is not 0, send "set_manual_speed()".
            # if not, then send "go_to_local_point_body_fixed()" to stay at this point
            if v_y != 0 or yaw_rate != 0:
                mini.set_manual_speed_body_fixed(vx=0, vy=v_y, vz=0, yaw_rate=yaw_rate)
                send_manual_speed = True
            elif send_manual_speed:
                send_manual_speed = False
                mini.go_to_local_point_body_fixed(x=0, y=0, z=0, yaw=0)

        cv2.imshow("marker_detection", frame)  # show frame

        key = cv2.waitKey(1)
        if key == 27:  # Exit if the ESC key is pressed
            cv2.destroyAllWindows()
            mini.land()
            mini.close_connection()
            del mini
            break
