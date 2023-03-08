import re
from datetime import datetime
import json
import cv2 as cv
import numpy as np


def parse_robot_program(mod_file):
    moves_j = re.compile(
        "MoveJ \[\[(-?\d*[.]?\d+),(-?\d*[.]?\d*),(-?\d*[.]?\d*)\],\[(-?\d*[.]?\d*),(-?\d*[.]?\d*),(-?\d*[.]?\d*),(-?\d*[.]?\d*)\],.*?;")
    with open(mod_file, 'r') as file:
        prog = file.read()
        return moves_j.findall(prog)


def write_hand_eye_transform(rvec, tvec, output_file=None):
    from realsensescan.eye_in_hand._encoder import NumpyEncoder
    if not output_file:
        output_file = f"Calibration_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(output_file, 'w') as f:
        json.dump({"rvec": rvec, "tvec": tvec}, f, indent=2, cls=NumpyEncoder)


def write_calibration_result(camera_matrix, dist_coeffs, image_size, output_file):
    calibration_result = {
        "Timestamp": datetime.now().isoformat(),
        "Image Size": image_size,
        "Principal Point": [camera_matrix[0][2], camera_matrix[1][2]],
        "Focal Length": [camera_matrix[0][0], camera_matrix[1][1]],
        "Distortion Coefficients": dist_coeffs[0].tolist()
    }

    with open(f"{output_file}.json", 'w') as file:
        json.dump(calibration_result, file, indent=2)


def camera_matrix_from_intrinsics(f, c):
    return np.array([
        [f[0], 0, c[0]],
        [0, f[1], c[1]],
        [0, 0, 1]
    ])


def load_camera_calibration(camera_calibration):
    with open(camera_calibration) as camera_calibration_json:
        camera_params = json.load(camera_calibration_json)
        f = camera_params["Focal Length"]
        c = camera_params["Principal Point"]
        camera_matrix = camera_matrix_from_intrinsics(f, c)
        print("Camera Matrix:\n", camera_matrix)

        dist_coeff = np.array(camera_params["Distortion Coefficients"])
        print("Camera Distortion Coefficients:\n", dist_coeff)

    return camera_matrix, dist_coeff


def load_charuco_board(charuco_parameters, verbose=True):
    with open(charuco_parameters) as charuco_params_json:
        charuco_params = json.load(charuco_params_json)
        dict_name = charuco_params["markers_dictionary"]
        print("Using ARUCO dictionary: [{}]".format(dict_name))

        num_x = charuco_params["num_X"]
        num_y = charuco_params["num_Y"]
        len_squares = charuco_params["len_squares"]
        len_markers = charuco_params["len_markers"]

        # aruco_dict = cv.aruco.getPredefinedDictionary(getattr(cv.aruco, dict_name))
        aruco_dict = cv.aruco.getPredefinedDictionary(dict_name)
        board = cv.aruco.CharucoBoard((num_x, num_y), len_squares, len_markers, aruco_dict)

        if verbose:
            board_img = board.generateImage(np.array(board.getChessboardSize()) * 70)
            cv.imshow("Board", board_img)

    return board, aruco_dict


# Copied from: https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3)

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix
