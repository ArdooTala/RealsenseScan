## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

import datetime
import cv2
import copy
import json
from realsensescan.cam_control.presets import Preset
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
from multiprocessing import Process
from datetime import datetime


class CamControl:
    def __init__(self):
        self.device = None
        self.cam_matrix = None
        self.camera = None

        self.roll_camera = False
        self.save_path = "pcs"
        self.pcs = {}

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        # Create a pipeline
        self.pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        self.config = None

        self.depth_scale = None

        self.pcd = o3d.geometry.PointCloud()

        # We will not display the background of objects more than
        #  clipping_distance_in_meters meters away
        self.clipping_distance_in_meters = 3  # 3 meter

    @staticmethod
    def load_camera_calibration(camera_calibration):
        with open(camera_calibration) as camera_calibration_json:
            camera_params = json.load(camera_calibration_json)
            f = camera_params["Focal Length"]
            c = camera_params["Principal Point"]
            camera_matrix = np.array([
                [f[0], 0, c[0]],
                [0, f[1], c[1]],
                [0, 0, 1]
            ])
            print("Camera Matrix:\n", camera_matrix)

            dist_coeff = np.array(camera_params["Distortion Coefficients"])
            print("Camera Distortion Coefficients:\n", dist_coeff)

        return camera_matrix, dist_coeff

    # def capture_point_cloud(self, config_file, calibration_file):
    #     with open(config_file) as cf:
    #         rs_cfg = o3d.t.io.RealSenseSensorConfig(json.load(cf))
    #
    #     rs = o3d.t.io.RealSenseSensor()
    #     rs.init_sensor(rs_cfg, 0, "")
    #     rs.start_capture(False)  # true: start recording with capture
    #
    #     cam_matrix = self.load_camera_calibration(calibration_file)[0]
    #
    #     intrinsic = o3d.camera.PinholeCameraIntrinsic(848, 480,
    #                                                   cam_matrix[0][0], cam_matrix[1][1],
    #                                                   cam_matrix[0][2], cam_matrix[1][2])
    #     for fid in range(150):
    #         im_rgbd = rs.capture_frame(True, True)  # wait for frames and align them
    #
    #         plt.subplot(1, 2, 1)
    #         plt.title('Redwood grayscale image')
    #         plt.imshow(im_rgbd.color)
    #         plt.subplot(1, 2, 2)
    #         plt.title('Redwood depth image')
    #         plt.imshow(im_rgbd.depth)
    #         plt.show()
    #
    #         pcd = o3d.t.geometry.PointCloud.create_from_rgbd_image(im_rgbd, intrinsic)
    #
    #         # Flip it, otherwise the pointcloud will be upside down
    #         pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    #
    #         o3d.visualization.draw_geometries([pcd, ])
    #
    #     rs.stop_capture()

    def get_intrinsic_matrix_from_camera(self, frame):
        intrinsics = frame.profile.as_video_stream_profile().intrinsics
        self.cam_matrix = o3d.camera.PinholeCameraIntrinsic(640, 480, intrinsics.fx,
                                                            intrinsics.fy, intrinsics.ppx,
                                                            intrinsics.ppy)
        return self.cam_matrix

    def config_camera(self, width=848, height=480, framerate=30):
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, framerate)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, framerate)

    def buffer_frame(self, pc):
        print("\n" * 10)
        pc = copy.deepcopy(self.pcd)
        now = datetime.now()
        self.pcs[now] = pc
        p = Process(
            target=o3d.io.write_point_cloud,
            args=(f"{self.save_path}/{now.strftime('%Y%m%d_%H%M%S')}.ply", pc))
        p.start()
        print("\n" * 10)
        #o3d.io.write_point_cloud(f"{self.save_path}/{now.strftime('%Y%m%d_%H%M%S')}.ply", self.pcd)

    def stream_camera(self):
        # Start streaming
        profile = self.pipeline.start(self.config)
        depth_sensor = profile.get_device().first_depth_sensor()

        # Using preset HighAccuracy for recording
        depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        self.depth_scale = depth_sensor.get_depth_scale()

        clipping_distance = self.clipping_distance_in_meters / self.depth_scale
        # print(depth_scale)

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        align = rs.align(align_to)

        # flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

        # Streaming loop
        frame_count = 0
        # try:
        while self.roll_camera:
            dt0 = datetime.now()
            # Get frameset of color and depth
            frames = self.pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            intrinsic = o3d.camera.PinholeCameraIntrinsic(
                self.get_intrinsic_matrix_from_camera(color_frame))

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = o3d.geometry.Image(
                np.array(aligned_depth_frame.get_data()))
            color_temp = np.asarray(color_frame.get_data())
            color_image = o3d.geometry.Image(color_temp)

            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image,
                depth_image,
                depth_scale=1.0 / self.depth_scale,
                depth_trunc=self.clipping_distance_in_meters,
                convert_rgb_to_intensity=False)
            temp = o3d.geometry.PointCloud.create_from_rgbd_image(
                rgbd_image, intrinsic)
            # temp.transform(flip_transform)
            self.pcd.points = temp.points
            self.pcd.colors = temp.colors

            print(self.pcd)

            if frame_count == 0:
                self.vis.add_geometry(self.pcd)

            self.vis.update_geometry(self.pcd)
            self.vis.poll_events()
            self.vis.update_renderer()

            process_time = datetime.now() - dt0
            print("FPS: " + str(1 / process_time.total_seconds()))
            frame_count += 1
        #
        # finally:
        #     self.pipeline.stop()
        # self.vis.destroy_window()

    def visualize_point_cloud(self, pcd):
        pass

    def kill_stream(self, _):
        self.roll_camera = False

    def __enter__(self):
        self.config_camera()
        self.roll_camera = True
        self.vis.register_key_callback(ord("S"), self.buffer_frame)
        self.vis.register_key_callback(ord("Q"), self.kill_stream)
        self.vis.create_window()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.pipeline.stop()
        finally:
            self.vis.destroy_window()


if __name__ == "__main__":
    with CamControl() as cam_control:
        # p = Process(target=cam_control.stream_camera)
        # p.start()
        cam_control.stream_camera()

# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2018-2021 www.open3d.org
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
# ----------------------------------------------------------------------------

# examples/python/reconstruction_system/sensors/realsense_pcd_visualizer.py

# pyrealsense2 is required.
# Please see instructions in https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python


def capture_rgb_camera(save_path):
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
            if k == ord("s"):
                cv2.imwrite(f"{save_path}/{i:03}-{datetime.now().strftime('%Y%m%d_%H%M%S')}.png", color_image)
                i += 1

    finally:
        pipeline.stop()
