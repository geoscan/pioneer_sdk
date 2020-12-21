from pioneer_sdk import Pioneer
import os
import sys
import numpy as np
import cv2
import yaml


number_of_samples = 15

number_of_hor_corners = 6
number_of_ver_corners = 9

counter = 1
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
obj_p = np.zeros((number_of_hor_corners*number_of_ver_corners, 3), np.float32)
obj_p[:, :2] = np.mgrid[0:number_of_ver_corners, 0:number_of_hor_corners].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
obj_points = []  # 3d point in real world space
img_points = []  # 2d points in image plane.

if __name__ == '__main__':
    pioneer_mini = Pioneer(logger=False)
    video_frame = bytes()
    if number_of_samples < 10:
        print('Algorithm need at least 10 samples')
        sys.exit(0)
    print('press p to take samples, or esc for exit')
    print('by default they will be stored in ./camera_samples folder and the result file will appear in ./result folder')
    print('Defined number of samples is %d' % number_of_samples)
    os.chdir(os.path.dirname(sys.argv[0]))  # script dir
    samples_folder = os.path.join(os.getcwd(), "camera_samples")
    yaml_folder = os.path.join(os.getcwd(), "result")
    if not os.path.isdir(samples_folder):
        os.mkdir(samples_folder)
    if not os.path.isdir(yaml_folder):
        os.mkdir(yaml_folder)
    while True:
        camera_frame = cv2.imdecode(np.frombuffer(pioneer_mini.get_raw_video_frame(), dtype=np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow('pioneer_camera_stream', camera_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            sys.exit(0)
        if key == ord('p'):
            print('Taking photo...')
            if not cv2.imwrite(os.path.join(samples_folder, 'frame_%d.jpg' % counter), camera_frame):
                raise Exception("Could not write image")
            img = cv2.imread(os.path.join(samples_folder, 'frame_%d.jpg' % counter))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (number_of_ver_corners, number_of_hor_corners), None)
            # If found, add object points, image points (after refining them)
            if ret:
                obj_points.append(obj_p)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                img_points.append(corners2)
                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (number_of_ver_corners, number_of_hor_corners), corners2, ret)
                cv2.imshow('img', img)
                print('There are %d photos left to take' % (number_of_samples - counter))
                counter += 1
                cv2.waitKey(500)
                cv2.destroyWindow('img')
            else:
                print('Bad sample, try again')

            if counter == number_of_samples + 1:
                cv2.destroyAllWindows()
                break

    """
    Performing camera calibration by
    passing the value of known 3D points (objpoints)
    and corresponding pixel coordinates of the
    detected corners (imgpoints)
    """
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
    print("Camera matrix : \n")
    print(mtx)
    print("dist : \n")
    print(dist)
    print("rvecs : \n")
    print(rvecs)
    print("tvecs : \n")
    print(tvecs)

    # transform the matrix and distortion coefficients to writable lists
    data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}

    # and save it to a file
    with open(os.path.join(yaml_folder, 'calibration_matrix.yaml'), "w") as f:
        yaml.dump(data, f)
