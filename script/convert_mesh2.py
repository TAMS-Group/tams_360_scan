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


def convert_pickle_to_rgb_image():
    fig_path = rospkg.RosPack().get_path("tams_head_mesh")
    meshes = glob.glob(fig_path+"/data/shot_20210812/save_*.pickle")
    for i, mesh in enumerate(meshes):
        with open(mesh, "rb") as handle:
            data = pickle.load(handle, encoding="latin1")
            rgb = data["rgb"]
            rbg_img = Image.fromarray(rgb)
            rbg_img.save(os.path.dirname(mesh)+"/rgb/"+mesh.split("/")[-1][:-6]+"jpg")


if __name__ == "__main__":
    convert_pickle_to_rgb_image()
