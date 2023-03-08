from realsensescan.eye_in_hand import helpers
import realsensescan.eye_in_hand.imageprocessing as iProc
import pathlib as pth
import cv2 as cv
import numpy as np


def preprocess_calibration_program(mod_file, images_dir, output_file):
    match = helpers.parse_robot_program(mod_file)
    images_dir = pth.Path(images_dir)

    with open(output_file, 'w') as pose_file:
        pose_file.write("imgName,x,y,z,q1,q2,q3,q4\n")

        for img, pose in zip(sorted(images_dir.glob("*.png")), match):
            print(img.name)
            print(pose)
            pose_file.write(f"{img.stem},{','.join(pose)}\n")

    return match


def preprocess_calibration_data(poses_csv, images_dir, camera_calibration, charuco_parameters, verbose=True):
    camera_matrix, dist_coeff = helpers.load_camera_calibration(camera_calibration)
    board, aruco_dict = helpers.load_charuco_board(charuco_parameters)
    images_dir = pth.Path(images_dir)

    calibration_model = {
        "r_gripper2base": [],
        "t_gripper2base": [],
        "r_target2cam": [],
        "t_target2cam": []
    }

    with open(poses_csv) as poses:
        poses.readline()
        for line in poses:
            line = line.split(',')
            image_file = line[0] + ".png"
            image_file = images_dir / image_file
            print(" Estimating Board Pose from: [{}] ".format(image_file.name).center(150, "="))
            if not image_file.exists():
                print("Image file does not exists: [{}]".format(image_file))
                continue
            img = cv.imread(str(image_file))
            transform = iProc.estimate_pose(img, camera_matrix, dist_coeff, board, aruco_dict, verbose=verbose)

            if not transform:
                print("Failed to estimate board pose!")
                continue

            tcp_pos = [float(num) / 1000 for num in line[1:4]]
            tcp_rot = [float(num) for num in line[4:]]
            if verbose:
                print(
                    "Robot Pose:\n\t{: .4f}, {: .4f}, {: .4f}\n\t{: f}, {: f}, {: f}, {: f}".format(*tcp_pos, *tcp_rot))

            calibration_model["r_gripper2base"].append(helpers.quaternion_rotation_matrix(tcp_rot))
            calibration_model["t_gripper2base"].append(tcp_pos)
            calibration_model["r_target2cam"].append(transform[0])
            calibration_model["t_target2cam"].append(transform[1])

    return calibration_model


def calibrate(calibration_model):
    print("#" * 150)
    print("Calibrating on {} images . . .".format(len(list(calibration_model.values())[0])))
    r_cam2gripper, t_cam2gripper = cv.calibrateHandEye(
        np.array(calibration_model["r_gripper2base"]),
        np.array(calibration_model["t_gripper2base"]),
        np.array(calibration_model["r_target2cam"]),
        np.array(calibration_model["t_target2cam"]),
        cv.CALIB_HAND_EYE_DANIILIDIS  # cv.CALIB_HAND_EYE_TSAI
    )

    return r_cam2gripper, t_cam2gripper


def generate_charuco_board(charuco_parameters, output_file):
    board, aruco_dict = helpers.load_charuco_board(charuco_parameters, verbose=False)
    board_img = board.generateImage(np.array(board.getChessboardSize()) * 70)

    cv.imshow("GeneratedBoard", board_img)
    cv.waitKey(0)

    cv.imwrite(output_file, board_img)


def calibrate_camera(images_dir, charuco_parameters, output_file):
    board, aruco_dict = helpers.load_charuco_board(charuco_parameters, verbose=False)
    images_dir = pth.Path(images_dir)

    corners_all = []  # Corners discovered in all images processed
    ids_all = []  # Aruco ids corresponding to corners discovered
    image_size = None  # Determined at runtime

    for img_file in images_dir.glob("*.png"):
        img = cv.imread(str(img_file))

        if not image_size:
            image_size = img.shape[1::-1]
            print(image_size)

        res = iProc.detect_markers(img, aruco_dict, board)

        if not res:
            print("Not able to detect a charuco board in image: {}".format(img_file))
            continue

        charuco_corners, charuco_ids = res

        corners_all.append(charuco_corners)
        ids_all.append(charuco_ids)

    if not image_size:
        print(
            "Calibration was unsuccessful. We couldn't detect charucoboards in any of the images supplied. Try changing the patternSize passed into Charucoboard_create(), or try different pictures of charucoboards.")
        return

    calibration, cameraMatrix, distCoeffs, rvecs, tvecs = cv.aruco.calibrateCameraCharuco(
        charucoCorners=corners_all,
        charucoIds=ids_all,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None)

    print(cameraMatrix)
    print(distCoeffs)

    helpers.write_calibration_result(cameraMatrix, distCoeffs, image_size, output_file)
