#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author     : Hongzhuo Liang 
# E-mail     : liang@informatik.uni-hamburg.de
# Description: 
# Date       : 02/08/2021: 5:26 PM
# File Name  : convert_mesh
import rospy
import glob
import numpy as np
import pickle
import rospkg
from PIL import Image
import os
from apriltag_ros.srv import AnalyzeSingleImage, AnalyzeSingleImageRequest
import yaml
from pyquaternion import Quaternion
import open3d as o3d
from grasp_tools.utils.utils import compute_xyz


class ConvertMesh:
    def __init__(self):
        self.base_path = rospkg.RosPack().get_path("tams_360_scan")

        rospy.wait_for_service("single_image_tag_detection")
        rospy.loginfo("got service: single_image_tag_detection")
        self.client = rospy.ServiceProxy("single_image_tag_detection", AnalyzeSingleImage)
        self.service = AnalyzeSingleImageRequest()
        self.service.camera_info.P = camera_params["P"]
        self.service.camera_info.distortion_model = "plumb_bob"
        self.model_type = rospy.get_param("~model_type", "gitarre")

    def convert_meshes(self):
        os.makedirs(self.base_path + f"/data/{self.model_type}/rgb/", exist_ok=True)
        meshes = glob.glob(self.base_path + f"/data/{self.model_type}/save_*.pickle")
        all_points = np.array([[0, 0, 0]])
        for i, mesh in enumerate(meshes):
            with open(mesh, "rb") as handle:
                data = pickle.load(handle, encoding="latin1")
                rgb = data["rgb"]
                depth = data["depth"]
            rbg_img = Image.fromarray(rgb)
            fig_save_path = os.path.dirname(mesh) + "/rgb/" + mesh.split("/")[-1][:-6] + "jpg"
            rbg_img.save(fig_save_path)
            self.service.full_path_where_to_get_image = fig_save_path
            self.service.full_path_where_to_save_image = fig_save_path[:-4] + "_result.jpg"
            pose = self.single_image_client()
            xyz = compute_xyz(depth, camera_params, flipud_indices=False, coppelia=False)
            xyz = xyz.reshape(-1, 3) / 1000.0
            ori = pose.orientation
            ori = Quaternion(ori.w, ori.x, ori.y, ori.z)
            pose_r = ori.rotation_matrix
            pose_t = np.array([pose.position.x, pose.position.y, pose.position.z])
            xyz = - pose_t.reshape(3) + xyz
            new_xyz = np.dot(pose_r.T, xyz.T).T
            all_points = np.vstack([all_points, new_xyz])
        if self.model_type == "head":
            all_points_crop = all_points[np.where(all_points[:, 2] > 0.006)]
            all_points_crop = all_points_crop[np.where(all_points_crop[:, 2] < 0.500)]
            all_points_crop = all_points_crop[np.where(all_points_crop[:, 1] < 0.100)]
            all_points_crop = all_points_crop[np.where(all_points_crop[:, 1] > -0.090)]
            all_points_crop = all_points_crop[np.where(all_points_crop[:, 0] > -0.20)]
            all_points_crop = all_points_crop[np.where(all_points_crop[:, 0] < 0.150)]
        elif self.model_type == "gitarre":
            all_points_crop = all_points[np.where(all_points[:, 2] > -0.1)]
            all_points_crop = all_points_crop[np.where(all_points_crop[:, 2] < 0.0100)]
            all_points_crop = all_points_crop[np.where(all_points_crop[:, 1] < 0.500)]
            all_points_crop = all_points_crop[np.where(all_points_crop[:, 1] > -0.50)]
            all_points_crop = all_points_crop[np.where(all_points_crop[:, 0] > -0.20)]
            all_points_crop = all_points_crop[np.where(all_points_crop[:, 0] < 0.25)]
        else:
            raise NotImplementedError
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points_crop)
        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=1.5)
        cl.estimate_normals()
        o3d.visualization.draw_geometries([cl])
        points = np.asarray(cl.points)
        # mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(cl, 4)  # this line fail to generate mesh
        # mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(cl)  # this line generate bad mesh
        with open("/tmp/tams_360_scan_points_1.pickle", "wb") as handle:
            pickle.dump(points, handle)
        np.savetxt("/tmp/tams_360_scan_points_1.xyz", points)

    def single_image_client(self):
        try:
            respond = self.client(self.service)
            dections = respond.tag_detections.detections
            for dection in dections :
                if dection.id[0] == 56 or len(dection.id) == 40:
                    pose = dection.pose.pose.pose
                    return pose
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))


if __name__ == "__main__":
    rospy.init_node("demo")
    pkg_path = rospkg.RosPack().get_path("tams_camera_config")
    with open(pkg_path + "/mechmind_camera/camera_info.yaml") as file:
        camera_params = yaml.safe_load(file)
    covert = ConvertMesh()
    covert.convert_meshes()
