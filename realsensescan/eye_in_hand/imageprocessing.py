import numpy as np
import cv2 as cv


def detect_markers(img, aruco_dict, board, verbose=True):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    corners, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict)

    img = cv.aruco.drawDetectedMarkers(img, corners)

    response, charuco_corners, charuco_ids = cv.aruco.interpolateCornersCharuco(
        markerCorners=corners,
        markerIds=ids,
        image=gray,
        board=board)

    if response < 10:
        return

    img = cv.aruco.drawDetectedCornersCharuco(img, charuco_corners, charuco_ids)

    if verbose:
        cv.imshow('Charuco board', img)
        cv.waitKey(1)

    return charuco_corners, charuco_ids


def estimate_pose(image, camera_matrix, dist_coeff, board, aruco_dict, verbose=True):
    frame = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    corners, ids, rejected_points = cv.aruco.detectMarkers(frame, aruco_dict)

    if corners is None or ids is None:
        return None
    if len(corners) != len(ids) or len(corners) == 0:
        return None

    if verbose:
        output = cv.aruco.drawDetectedMarkers(image, corners)  # , ids)

    try:
        ret, c_corners, c_ids = cv.aruco.interpolateCornersCharuco(corners, ids, frame, board)

        if verbose:
            output = cv.aruco.drawDetectedCornersCharuco(output, c_corners)  # , c_ids)

        if ret < 10:
            if verbose:
                cv.imshow("Kir", output)
                cv.waitKey(0)
            return None

        rvec = (0, 0, 0)
        tvec = (0, 0, 0)
        ret, p_rvec, p_tvec = cv.aruco.estimatePoseCharucoBoard(
            c_corners, c_ids, board, camera_matrix, dist_coeff, rvec, tvec)

        if verbose:
            print('Translation:\n{0}'.format(p_tvec))
            print('Rotation:\n{0}'.format(p_rvec))
            print('Distance from cameras:\t{0} m'.format(np.linalg.norm(p_tvec)))

        if p_rvec is None or p_tvec is None:
            if verbose:
                cv.imshow("Kir", output)
                cv.waitKey(0)
            return None
        if np.isnan(p_rvec).any() or np.isnan(p_tvec).any():
            if verbose:
                cv.imshow("Kir", output)
                cv.waitKey(0)
            return None

        if verbose:
            output = cv.drawFrameAxes(output, camera_matrix, dist_coeff, p_rvec, p_tvec, 0.1)

    except cv.error as e:
        print(e)
        return None

    if verbose:
        cv.imshow("Kir", output)
        cv.waitKey(1)

    return p_rvec, p_tvec
