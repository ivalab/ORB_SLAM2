#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file Run_TUM_Rgbd_ROS_node.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 01-02-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""


# This script is to run all the experiments in one program

import os
import subprocess
import time
from pathlib import Path
from dataclasses import dataclass


# ----------------------- THE FOLLOWINGS ARE FIXED -----------------------------
DATASET_DICT = {
    "tum-rgbd": [
        "rgbd_dataset_freiburg2_desk",
        "rgbd_dataset_freiburg2_desk_with_person",
        "rgbd_dataset_freiburg3_long_office_household",
    ],
    "cid": [
        "office_1",
        "floor13_1",
    ],
}
DATA_DIR_ROOT = Path("/mnt/DATA/datasets/")
OUTPUT_DIR_ROOT = Path("/mnt/DATA/experiments/semantic/")
# ------------------------------- END ------------------------------------------


@dataclass
class MethodMetaData:
    """_summary_"""

    name: str
    feature_num: int
    label: str


METHODS = [
    MethodMetaData("orb2", 800, "ORB2"),
    MethodMetaData("orb2", 1000, "ORB2"),
]

DATASETS = ["tum-rgbd"]
ROUNDS = 10
SPEEDS = [1.0]  # , 2.0, 3.0, 4.0, 5.0]  # x
SLEEP_TIME = 1  # 10 # 25
ENABLE_VIEWER = False
ENABLE_LOGGING = 1

ORB3_PATH = os.path.join(os.environ["HOME"], "closedloop_ws/src/ORB_SLAM2")
CONFIG_PATH = os.path.join(ORB3_PATH, "Examples/RGB-D")

SEQ_SETTINGS = {
    "rgbd_dataset_freiburg2_desk": "TUM2.yaml",
    "rgbd_dataset_freiburg2_desk_with_person": "TUM2.yaml",
    "rgbd_dataset_freiburg3_long_office_household": "TUM3.yaml",
}


# ----------------------------------------------------------------------------------------------------------------------
class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    ALERT = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


for method_idx, method in enumerate(METHODS):
    for data_idx, data_name in enumerate(DATASETS):
        sequences = DATASET_DICT[data_name]
        for speed_idx, speed in enumerate(SPEEDS):
            for round_idx in range(1, ROUNDS + 1):
                method_dir = (
                    OUTPUT_DIR_ROOT
                    / data_name
                    / method.name
                    / f"_Speedx{speed}/ObsNumber_{method.feature_num}_Round{round_idx}"
                )
                method_dir.mkdir(exist_ok=True, parents=True)
                for seq_idx, seq_name in enumerate(sequences):
                    print(
                        bcolors.OKGREEN
                        + f"Seq: {seq_name}; Feature: {method.feature_num}; Speed: {speed}; Round: {round_idx}"
                        + bcolors().ENDC
                    )

                    # hardcode, might try other feature number
                    file_setting = ""
                    if data_name == "tum-rgbd":
                        file_setting = os.path.join(
                            CONFIG_PATH, SEQ_SETTINGS[seq_name]
                        )
                    elif data_name == "cid":
                        file_setting = os.path.join(CONFIG_PATH, "CID_yaml/pinhole_depth.yaml")
                    if not os.path.exists(file_setting):
                        print(f"Failed to find the setting file: {file_setting}.")
                        exit(-1)

                    file_vocab = os.path.join(ORB3_PATH, "Vocabulary/ORBvoc.txt")

                    file_traj = str(method_dir / seq_name)
                    file_log = "> " + file_traj + "_logging.txt" if ENABLE_LOGGING else ""
                    file_data = DATA_DIR_ROOT / data_name / "rosbags" / seq_name
                    file_rosbag = str(file_data) + ".bag"
                    node_name = "RGBD"

                    # compose cmd
                    # cmd_slam = str(
                    #     "roslaunch gf_orb_slam2 slam.launch"
                    #     + " voc_filename:="
                    #     + file_vocab
                    #     + " settings_filename:="
                    #     + file_setting
                    #     + " good_feature_num:="
                    #     + str(int(num_gf))
                    #     + " do_vis:="
                    #     + str(EnableViewer)
                    #     + " sub_image0_topic:=/cam0/image_raw"
                    #     + " sub_image1_topic:=/cam1/image_raw"
                    #     + " output_dir:="
                    #     + Experiment_dir
                    #     + " dataset:="
                    #     + SeqName
                    #     + file_log
                    # )

                    cmd_slam = str(
                        "rosrun ORB_SLAM2 RGBD "
                        + file_vocab
                        + " "
                        + file_setting
                        + " "
                        + str(int(method.feature_num))
                        + " "
                        + str(1 if ENABLE_VIEWER else 0)
                        + " /camera/rgb/image_color /camera/depth/image "
                        + file_traj
                        + " "
                        + file_log
                    )

                    cmd_rosbag = "rosbag play " + file_rosbag + " -r " + str(speed)  # + ' -u 20'

                    print(bcolors.WARNING + "cmd_slam: \n" + cmd_slam + bcolors.ENDC)

                    print(bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC)
                    subprocess.Popen(cmd_slam, shell=True)

                    print(bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC)
                    time.sleep(SLEEP_TIME * 3)

                    print(bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC)
                    proc_bag = subprocess.call(cmd_rosbag, shell=True)

                    print(bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC)
                    subprocess.call(f"rosnode kill /{node_name}", shell=True)
                    time.sleep(SLEEP_TIME * 3)
                    # print bcolors.OKGREEN + "Saving the map to file system" + bcolors.ENDC
                    # time.sleep(15)
                    subprocess.call("pkill RGBD", shell=True)
