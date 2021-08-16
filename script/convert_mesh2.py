#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Hongzhuo Liang 
# E-mail     : liang@informatik.uni-hamburg.de
# Description: 
# Date       : 02/08/2021: 5:26 PM
# File Name  : convert_mesh
import rospy
import glob
import numpy as np
from grasp_tools.utils.utils import show_points
import pickle
from grasp_tools.utils.utils import compute_xyz
import matplotlib.pyplot as plt
import rospkg
from PIL import Image
import os
from apriltag_ros.srv import AnalyzeSingleImage, AnalyzeSingleImageRequest
import yaml
from pyquaternion import Quaternion


class ConvertMesh:
    def __init__(self):
        self.base_path = rospkg.RosPack().get_path("tams_head_mesh")

        rospy.wait_for_service("single_image_tag_detection")
        rospy.loginfo("got service: single_image_tag_detection")
        self.client = rospy.ServiceProxy("single_image_tag_detection", AnalyzeSingleImage)
        self.service = AnalyzeSingleImageRequest()
        self.service.camera_info.P = camera_params["P"]
        self.service.camera_info.distortion_model = "plumb_bob"

    def convert_meshes(self):
        meshes = glob.glob(self.base_path+"/data/shot_20210812/save_*.pickle")
        all_points = np.array([[0, 0, 0]])
        for i, mesh in enumerate(meshes):
            with open(mesh, "rb") as handle:
                data = pickle.load(handle, encoding="latin1")
                rgb = data["rgb"]
                depth = data["depth"]
            rbg_img = Image.fromarray(rgb)
            fig_save_path = os.path.dirname(mesh)+"/rgb/"+mesh.split("/")[-1][:-6]+"jpg"
            rbg_img.save(fig_save_path)
            self.service.full_path_where_to_get_image = fig_save_path
            self.service.full_path_where_to_save_image = fig_save_path[:-4]+"_result.jpg"
            pose = self.single_image_client()
            xyz = compute_xyz(depth, camera_params, flipud_indices=False, coppelia=False)
            xyz = xyz.reshape(-1, 3)
            ori = pose.orientation
            ori = Quaternion(ori.w, ori.x, ori.y, ori.z)
            pose_r = ori.rotation_matrix
            pose_t = np.array([pose.position.x, pose.position.y, pose.position.z])
            xyz = pose_t.reshape(3) - xyz
            new_xyz = np.dot(pose_r.T, xyz.T).T
            all_points = np.vstack([all_points, new_xyz])
            if i == 2:
                show_points(all_points, frame_size=100.5)
                exit()

    def single_image_client(self):
        try:
            respond = self.client(self.service)
            pose = respond.tag_detections.detections[0].pose.pose.pose
            return pose
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))


if __name__ == "__main__":
    rospy.init_node("demo")
    pkg_path = rospkg.RosPack().get_path("tams_camera_config")
    with open(pkg_path+"/mechmind_camera/camera_info.yaml") as file:
        camera_params = yaml.load(file)
    covert = ConvertMesh()
    covert.convert_meshes()
