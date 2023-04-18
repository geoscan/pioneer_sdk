#!/usr/bin/python3

"""
Performs camera calibration using checkerboard pattern. Saves calibration
values into a file.
More: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
"""

import cv2
import numpy as np
import glob
import time
from pioneer_sdk import Camera


def get_images_from_folder(folder_name, file_type="*.jpg"):
    """
    Retreive images from a directory using a provided path (`folder_name`)
    """
    images_list = glob.glob(folder_name + "/" + file_type)
    images = []
    for fname in images_list:
        img = cv2.imread(fname)
        images.append(img)
    return images


def get_images_from_drone_camera(camera, save=False):
    """
    Retreive images from a drone's camera
    """
    images = []
    print("Press '1' to take frame\nPress ESC or 'q' to finish")
    while True:
        frame = camera.get_cv_frame()
        cv2.imshow("frames", frame)

        key = cv2.waitKey(1)
        if key == 27 or key == ord("q"):
            cv2.destroyAllWindows()
            return images
        if key == ord("1"):
            images.append(frame)
            print("Frame #" + str(len(images)))
            if save:
                cv2.imwrite("frame_" + str(len(images)) + ".jpg", frame)
            time.sleep(1)


def get_images_from_computer_camera(capture, save=False):
    """
    Retreive images using the PC's camera device
    """
    images = []
    print("Press '1' to take frame\nPress ESC or 'q' to finish")
    while True:
        ret, frame = capture.read()
        cv2.imshow("frames", frame)

        key = cv2.waitKey(1)
        if key == 27 or key == ord("q"):
            cv2.destroyAllWindows()
            return images
        if key == ord("1"):
            images.append(frame)
            print("Frame #" + str(len(images)))
            if save:
                cv2.imwrite("frame_" + str(len(images)) + ".jpg", frame)
            time.sleep(1)


def calibrate(images):
    # Defining the dimensions of checkerboard
    CHECKERBOARD = (6, 9)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints = []

    # Defining the world coordinates for 3D points
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)

    # Extracting path of individual image stored in a given directory
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(
            gray,
            CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )

        if ret == True:
            objpoints.append(objp)
            # refining pixel coordinates for given 2d points.
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)

        cv2.imshow("img", img)
        cv2.waitKey(0)

    cv2.destroyAllWindows()

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    return mtx, dist


def save_coefficients(mtx, dist, path):
    """Save the camera matrix and the distortion coefficients to given path/file."""
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write("mtx", mtx)
    cv_file.write("dist", dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()


def load_coefficients(path):
    """Loads camera matrix and distortion coefficients."""
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode("mtx").mat()
    dist_coeffs = cv_file.getNode("dist").mat()

    cv_file.release()
    return camera_matrix, dist_coeffs


def main():
    """
    Acquires images from the provided image source, calibrates the camera.

    This particular example uses the drone's camera as the image source.
    However, you can change the source by replacing
    `get_images_from_computer_camera` w/ another function.

    Examples:
    ```
    # Load images from a directory
    images = get_images_from_folder('images')
    # ...
    ```

    ```
    # Capture the images from a camera device
    capture = cv2.VideoCapture(0)
    images = get_images_from_computer_camera(capture)
    capture.release()
    # ...
    ```
    """

    # Acquire images from the drone's camera
    camera = Camera()
    images = get_images_from_drone_camera(camera)

    mtx, dist = calibrate(images)

    save_coefficients(mtx, dist, "data.yml")


if __name__ == "__main__":
    main()
