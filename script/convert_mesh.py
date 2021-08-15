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
from pupil_apriltags import Detector
import matplotlib.pyplot as plt
import rospkg
import yaml


def rgb2gray(rgb):
    r, g, b = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    gray = 0.2989 * r + 0.5870 * g + 0.1140 * b
    gray = np.array(gray, dtype=np.uint8)
    return gray


def main():
    fig_path = rospkg.RosPack().get_path("tams_head_mesh")
    meshes = glob.glob(fig_path+"/data/shot_20210802/save_*.pickle")
    all_points = np.array([[0, 0, 0]])
    for i, mesh in enumerate(meshes):
        with open(mesh, "rb") as handle:
            data = pickle.load(handle, encoding="latin1")
            rgb = data["rgb"]
            gray = rgb2gray(rgb)
            depth = data["depth"]

            xyz = compute_xyz(depth, camera_params, flipud_indices=False, coppelia=False)
            xyz = xyz.reshape(-1, 3)
            cp = [camera_params["fx"], camera_params["fy"], camera_params["x_offset"], camera_params["y_offset"]]
            tags = at_detector.detect(gray, estimate_tag_pose=True, camera_params=cp, tag_size=160)
            pose_r = tags[0].pose_R
            pose_t = tags[0].pose_t
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = pose_r
            transformation_matrix[:3, 3] = pose_t.reshape(3)
            xyz = pose_t.reshape(3) - xyz

            new_xyz = np.dot(pose_r.T, xyz.T).T
            all_points = np.vstack([all_points, new_xyz])
    all_points_crop = all_points[np.where(all_points[:, 2] > 0)]
    all_points_crop = all_points_crop[np.where(all_points_crop[:, 2] < 500)]
    all_points_crop = all_points_crop[np.where(all_points_crop[:, 1] < 300)]
    all_points_crop = all_points_crop[np.where(all_points_crop[:, 0] > 100)]
    all_points_crop = all_points_crop[np.where(all_points_crop[:, 0] < 500)]
    show_points(all_points_crop, frame_size=1)
    with open("/tmp/tams_head_points_20210802.pickle", "wb") as handle:
        pickle.dump(all_points_crop, handle)
    np.savetxt("/tmp/tams_head_points_20210802.xyz", all_points_crop)


if __name__ == "__main__":
    rospy.init_node("demo")
    pkg_path = rospkg.RosPack().get_path("tams_camera_config")
    with open(pkg_path+"/mechmind_camera/camera_info.yaml") as file:
        camera_params = yaml.load(file)
    at_detector = Detector(families="tag36h11", nthreads=1, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1,
                           decode_sharpening=0.25, debug=0)
    print("calculate points for the first shot")
    main()
