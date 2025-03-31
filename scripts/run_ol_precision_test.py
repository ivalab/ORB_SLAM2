#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file run_ol_precision_test.py
@author Yanwei Du (yanwei.du@gatech.edu)
@date 01-30-2025
@version 1.0
@license Copyright (c) 2025
@desc None
"""


# This script is to run all the experiments in one program

import os
import subprocess
import time

DATA_ROOT = os.path.join(os.environ["HOME"], "slam_ws/data/euroc/rosbags")
# DATA_ROOT = "/mnt/DATA/datasets/euroc/rosbags"
SeqNameList = [
    "MH_01_easy",
    "MH_02_easy",
    "MH_03_medium",
    "MH_04_difficult",
    "MH_05_difficult",
    "V1_01_easy",
    "V1_02_medium",
    "V1_03_difficult",
    "V2_01_easy",
    "V2_02_medium",
    "V2_03_difficult",
]

Result_root = os.path.join(os.environ["HOME"], f"slam_ws/results/experiments/openloop/laptop/precision/regular/orb3")
# Result_root = "/tmp/gfgg"

# Number_GF_List = [400, 800, 1000, 1500]
Number_GF_List = [1200]
NumRepeating = 5  # 10 # 20 #  5 #
NumLoops = 10
EnableSlomo = False
SpeedPool = [1.0]  # , 4.0, 5.0]  # x
SleepTime = 1  # 10 # 25
EnableViewer = False
EnableLoggingToFile = 0

ORB3_PATH = os.path.join(os.environ["HOME"], "closedloop_ws/src/ORB_SLAM3")
ConfigPath = os.path.join(os.environ["HOME"], "closedloop_ws/src/ORB_Data")


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


for speed in SpeedPool:

    Result_root_speed = Result_root + "_Speedx" + str(speed)

    for ri, num_gf in enumerate(Number_GF_List):

        Experiment_prefix = "ObsNumber_" + str(int(num_gf))

        for iteration in range(0, NumRepeating):

            Experiment_dir = os.path.join(Result_root_speed, Experiment_prefix + "_Round" + str(iteration + 1))

            # mkdir for pose traj
            cmd_mkdir = "mkdir -p " + Experiment_dir
            subprocess.call(cmd_mkdir, shell=True)

            for sn, sname in enumerate(SeqNameList):

                SeqName = SeqNameList[sn]  # + '_blur_5'

                print(
                    bcolors.OKGREEN
                    + f"Seq: {SeqName}; Feature: {num_gf}; Speed: {speed}; Round: {iteration + 1}"
                    + bcolors().ENDC
                )

                # hardcode, might try other feature number
                file_setting = os.path.join(ConfigPath, "EuRoC_yaml/EuRoC_stereo_lmk800.yaml")

                file_vocab = os.path.join(ConfigPath, "ORBvoc.bin")

                file_traj = os.path.join(Experiment_dir, SeqName)
                file_dummy_map = file_traj + "_dummy_map.txt"
                file_log = " > " + file_traj + "_logging.txt" if EnableLoggingToFile else ""
                file_data = os.path.join(DATA_ROOT, SeqName)
                file_rosbag = os.path.join(DATA_ROOT, SeqName + ".bag")
                file_timestamp = os.path.join(file_data, "times.txt")
                gt_pose_filename = os.path.join(ConfigPath, f"EuRoC_POSE_GT/{SeqName}_cam0.txt")

                # compose cmd
                cmd_slam = str(
                    "rosrun ORB_SLAM3 Stereo "
                    + file_vocab
                    + " "
                    + file_setting
                    + " "
                    + str(int(num_gf))
                    + " "
                    + "false"
                    + " "
                    + str(EnableViewer)
                    + " "
                    + "/cam0/image_raw /cam1/image_raw"
                    + " "
                    + file_traj
                    + " "
                    + file_log
                )

                print(bcolors.WARNING + "cmd_slam: \n" + cmd_slam + bcolors.ENDC)

                print(bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC)
                subprocess.Popen(cmd_slam, shell=True)

                print(bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC)
                time.sleep(SleepTime * 3)

                for loop_index in range(NumLoops):
                    playspeed = speed
                    if EnableSlomo and loop_index == 0:
                        playspeed = 1.0
                    cmd_rosbag = "rosbag play " + file_rosbag + " -r " + str(playspeed)  # + " -u 5"
                    print(bcolors.OKGREEN + f"Loop {loop_index}: launching rosbag ... " + bcolors.ENDC)
                    proc_bag = subprocess.call(cmd_rosbag, shell=True)
                    time.sleep(SleepTime * 3)

                print(bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC)
                subprocess.call("rosnode kill /visual_slam", shell=True)
                time.sleep(SleepTime)
                # print bcolors.OKGREEN + "Saving the map to file system" + bcolors.ENDC
                # time.sleep(15)
