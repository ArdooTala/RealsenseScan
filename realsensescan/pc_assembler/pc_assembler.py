import copy
from datetime import datetime
import re
from pathlib import Path
import open3d as o3d
import numpy as np
from realsensescan.cam_control import cam_control


class PointCloudAssembler:
    def __init__(self, mod_file, rows, cols):
        self.rows = rows
        self.cols = cols
        self.pcs = []
        self.pcs_down = []
        self.extrinsics = []
        self.robot_mod_file = mod_file
        self.save_path = "pcs/assembled"
        self.pose_graph = None

        self.voxel_size = 0.005
        self.max_correspondence_distance_coarse = self.voxel_size * 10
        self.max_correspondence_distance_fine = self.voxel_size * 1.5

    def load_point_clouds(self, pc_path):
        save_path = Path(pc_path) / "assembled"
        save_path.mkdir(exist_ok=True)
        self.save_path = str(save_path)

        cap_pcs = {}

        self.pcs = [pc for _, pc in sorted(cap_pcs.items(), key=lambda x: x[0])]
        pth = Path(pc_path)
        for pc_path in pth.glob('*.ply'):
            pcd = o3d.io.read_point_cloud(str(pc_path))

            # ### Temp Code
            # flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
            # pcd.transform(flip_transform)
            # ###

            cap_pcs[str(pc_path)] = pcd
        for i in cap_pcs.items():
            print(i)
        self.pcs = [pc for _, pc in sorted(cap_pcs.items(), key=lambda x: x[0])]

    def save_point_cloud(self, pc):
        o3d.io.write_point_cloud(f"{self.save_path}/assembled-{datetime.now().strftime('%Y%m%d_%H%M%S')}.ply", pc)

    def save_point_clouds(self, voxel_size=None):
        pcd_combined = o3d.geometry.PointCloud()
        for point_id in range(len(self.pcs)):
            pcd_combined += self.pcs[point_id]
        if voxel_size:
            pcd_combined = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
        self.save_point_cloud(pcd_combined)
        o3d.visualization.draw_geometries([pcd_combined, ])

    def capture_clouds(self, save_dir):
        path = Path(f"{save_dir}/{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        path.mkdir()

        save_path = path / "assembled"
        save_path.mkdir(exist_ok=True)
        self.save_path = str(save_path)

        print(path)
        print(save_path)

        with cam_control.CamControl() as cc:
            cc.save_path = str(path)
            cc.stream_camera()

        cap_pcs = cc.pcs

        for i in cap_pcs.items():
            print(i)
        self.pcs = [pc for _, pc in sorted(cap_pcs.items(), key=lambda x: x[0])]

    def assemble_cloud(self):
        self.parse_robot_program(self.robot_mod_file)
        self.process_point_clouds()
        # o3d.visualization.draw_geometries(self.pcs)

        # with o3d.utility.VerbosityContextManager(
        #         o3d.utility.VerbosityLevel.Debug) as cm:
        #     self.create_pose_graph()

    def parse_robot_program(self, mod_file):
        print("="*150)
        moves_j = re.compile(
            "MoveJ \[\[(-?\d*[.]?\d+),(-?\d*[.]?\d*),(-?\d*[.]?\d*)\],\[(-?\d*[.]?\d*),(-?\d*[.]?\d*),(-?\d*[.]?\d*),(-?\d*[.]?\d*)\],.*?;")
        with open(mod_file, 'r') as file:
            prog = file.read()
            poses = moves_j.findall(prog)
        print(f"{len(poses)} MoveJ lines found.")
        for pose in poses:
            print(pose)
            T = np.eye(4)
            T[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion(pose[3:])
            T[0:3, 3] = np.array(pose[:3], dtype=np.float32) / 1000
            print(T)
            print("___")
            self.extrinsics.append(T)
        print(f"{len(self.extrinsics)} matrices created.")

    def process_point_clouds(self):
        if not self.pcs:
            raise Exception("No Point Clouds . . . !")
        if not self.extrinsics:
            raise Exception("No Transforms . . . !")
        if len(self.pcs) != len(self.extrinsics):
            raise Exception(
                f"Number of Point Clouds ({len(self.pcs)}) and Transforms ({len(self.extrinsics)}) don't match.")

        for t, pc in zip(self.extrinsics, self.pcs):
            pc.transform(t)
            t = np.identity(4)

    def create_pose_graph(self):
        pose_graph = o3d.pipelines.registration.PoseGraph()

        for _ in range(len(self.pcs)):
            # TODO: Use robot pose as the transform to solve the transformations in one go.
            pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.identity(4)))

        if not self.pcs_down:
            self.preprocess_point_clouds()

        for i in range(len(self.pcs) - 1):
            print(f"Adding Pose Graph Edge {i} -> {i+1}")
            if i % self.cols == self.cols - 1:
                continue

            source_pc = self.pcs_down[i]
            target_pc = self.pcs_down[i+1]

            transformation_icp, information_icp = self.pairwise_registration(source_pc, target_pc)

            pose_graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(i, i+1,
                                                         transformation_icp,
                                                         information_icp,
                                                         uncertain=False))

        for i in range(len(self.pcs) - self.cols):
            print(f"Adding Pose Graph Edge {i} -> {i + self.cols}")
            source_pc = self.pcs_down[i]
            target_pc = self.pcs_down[i+self.cols]

            transformation_icp, information_icp = self.pairwise_registration(source_pc, target_pc)

            pose_graph.edges.append(
                o3d.pipelines.registration.PoseGraphEdge(i, i+self.cols,
                                                         transformation_icp,
                                                         information_icp,
                                                         uncertain=False))

        self.pose_graph = pose_graph
        return pose_graph

    def pairwise_registration(self, source, target):
        print("Apply point-to-plane ICP")

        # self.draw_registration_result(source, target, np.identity(4))

        icp_coarse = o3d.pipelines.registration.registration_icp(
            source, target, self.max_correspondence_distance_coarse, np.identity(4),
            o3d.pipelines.registration.TransformationEstimationPointToPlane())

        icp_fine = o3d.pipelines.registration.registration_icp(
            source, target, self.max_correspondence_distance_fine,
            icp_coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())

        # self.draw_registration_result(source, target, icp_fine.transformation)

        transformation_icp = icp_fine.transformation
        information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            source, target, self.max_correspondence_distance_fine,
            icp_fine.transformation)
        return transformation_icp, information_icp

    def preprocess_point_clouds(self):
        for pcd in self.pcs:
            print(":: Downsample with a voxel size %.3f." % self.voxel_size)
            pcd_down = pcd.voxel_down_sample(self.voxel_size)

            radius_normal = self.voxel_size * 2
            print(":: Estimate normal with search radius %.3f." % radius_normal)
            pcd_down.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

            self.pcs_down.append(pcd_down)
        return self.pcs_down

    def optimize_pose_graph(self):
        print("Optimizing Pose Graph".center(200, '='))
        if not self.pose_graph:
            raise Exception("Pose Graph not created yet.")

        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=self.max_correspondence_distance_fine,
            edge_prune_threshold=0.25,
            reference_node=0)
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            o3d.pipelines.registration.global_optimization(
                self.pose_graph,
                o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
                o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
                option)

        # TODO: Temp Code
        print("Transform points and display")
        for point_id in range(len(self.pcs)):
            print(self.pose_graph.nodes[point_id].pose)
            self.pcs[point_id].transform(self.pose_graph.nodes[point_id].pose)
        # o3d.visualization.draw_geometries(self.pcs)

    def draw_registration_result(self, source, target, transformation):
        source_temp = copy.deepcopy(source)
        target_temp = copy.deepcopy(target)
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(transformation)
        o3d.visualization.draw_geometries([source_temp, target_temp])


if __name__ == "__main__":
    pc_ass = PointCloudAssembler("../../data/robot_files/GridScan_T_ROB1.mod", rows=3, cols=5)
    pc_ass.capture_clouds("../../data/point_clouds")
    # pc_ass.load_point_clouds("pcs/20230310_174142")
    pc_ass.assemble_cloud()
    o3d.visualization.draw_geometries(pc_ass.pcs)
    pc_ass.create_pose_graph()
    pc_ass.optimize_pose_graph()
    o3d.visualization.draw_geometries(pc_ass.pcs)
    pc_ass.save_point_clouds()
