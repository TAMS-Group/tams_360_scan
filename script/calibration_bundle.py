#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author     : Hongzhuo Liang 
# E-mail     : liang@informatik.uni-hamburg.de
# Description: 
# Date       : 15/08/2021: 1:38 PM
# File Name  : calibration_bundle

from __future__ import print_function
from __future__ import division
import rospy
import rosbag
import numpy as np
import os
import yaml
import rospkg


class Extractor:
    def __init__(self):
        pkg_path = rospkg.RosPack().get_path("tams_head_mesh")
        self.bag_path = os.path.join(pkg_path, "data/calibration.bag")
        self.new_tag_setting_path = os.path.join(pkg_path, "config/tags_test.yaml")
        self.original_tag_setting_path = os.path.join(pkg_path, "config/tags.yaml")
        bag = rosbag.Bag(self.bag_path)
        self.read_tag_msg(bag)
        bag.close()

    def read_tag_msg(self, bag):
        positions = {}
        orientations = {}
        for topic, msg, t in bag.read_messages(topics="/tag_detections"):
            for tag in msg.detections:
                position = tag.pose.pose.pose.position
                orientation = tag.pose.pose.pose.orientation
                position = np.array([position.x, position.y, position.z])
                orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
                if tag.id[0] in positions.keys():
                    positions[tag.id[0]] = np.vstack([positions[tag.id[0]], position])
                    orientations[tag.id[0]] = np.vstack([orientations[tag.id[0]], orientation])
                else:
                    positions[tag.id[0]] = position
                    orientations[tag.id[0]] = orientation
        tags = {}
        orientation_list = []
        position_list = []
        for key in positions.keys():
            tags[key] = {"position": np.mean(positions[key], axis=0),
                         "orientation": np.mean(orientations[key], axis=0)}
            orientation_list.append(np.mean(orientations[key], axis=0))
            position_list.append(np.mean(positions[key], axis=0))
        orientation_mean = np.mean(np.array(orientation_list), axis=0)
        position_mean = np.mean(np.array(position_list), axis=0)
        with open(self.original_tag_setting_path, "r") as stream:
            try:
                config = yaml.safe_load(stream)
            except ModuleNotFoundError:
                config = yaml.load(stream)
        layout = []
        for i in range(len(config["standalone_tags"])):
            tag = config["standalone_tags"][i]
            tag["x"] = float(tags[tag["id"]]["position"][0] - position_mean[0])
            tag["y"] = float(tags[tag["id"]]["position"][1] - position_mean[1])
            tag["z"] = float(tags[tag["id"]]["position"][2] - position_mean[2])
            tag["qw"] = float(orientation_mean[3])
            tag["qx"] = float(orientation_mean[0])
            tag["qy"] = float(orientation_mean[1])
            tag["qz"] = float(orientation_mean[2])
            layout.append(tag)
        new_config = {"tag_bundles": [{"name": "head_mesh_bundle", "layout": layout}]}
        with open(self.new_tag_setting_path, "w") as outfile:
            yaml.dump(new_config, outfile, default_flow_style=True)
        print("save file to {}".format(self.new_tag_setting_path))


if __name__ == "__main__":
    print("Warning: this code is not finished")
    print("rewrite according to "
          "https://github.com/AprilRobotics/apriltag_ros/blob/master/apriltag_ros/scripts/calibrate_bundle.m")
    exit()
    rospy.init_node("tag_bundle_calibration", anonymous=True)
    Extractor()

