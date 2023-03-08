import json
import open3d as o3d


with open("rs_default_config.json") as cf:
    rs_cfg = o3d.t.io.RealSenseSensorConfig(json.load(cf))

rs = o3d.t.io.RealSenseSensor()
rs.init_sensor(rs_cfg, 0, "")
rs.start_capture(False)  # true: start recording with capture
for fid in range(150):
    im_rgbd = rs.capture_frame(True, True)  # wait for frames and align them
    # process im_rgbd.depth and im_rgbd.color

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        im_rgbd,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    o3d.visualization.draw_geometries([pcd], zoom=0.5)

rs.stop_capture()
