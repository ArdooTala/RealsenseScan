## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

import datetime
import pyrealsense2 as rs
import numpy as np
import cv2


def capture_rgb_camera(save_path, camera_matrix=None, dist_coeff=None, board=None, aruco_dict=None, verbose=False):
    pipeline = rs.pipeline()
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    pipeline.start(config)

    try:
        i = 0
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            # iProc.estimate_pose(color_image, camera_matrix, dist_coeff, board, aruco_dict)

            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            k = cv2.waitKey(1)
            if k == ord('s'):
                cv2.imwrite(f"{save_path}/{i:03}-{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png", color_image)
                i += 1

    finally:
        pipeline.stop()
