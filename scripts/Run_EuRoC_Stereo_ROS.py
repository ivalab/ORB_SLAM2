#!/usr/bin/env python
# -*-coding:utf-8 -*-
"""
@file Run_EuRoC_Stereo_ROS.py
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
import signal

# DATA_ROOT = os.path.join(os.environ["HOME"], "slam_ws/data/euroc/rosbags")
DATA_ROOT = "/mnt/DATA/datasets/euroc/rosbags"
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

# Result_root = os.path.join(os.environ["HOME"], "slam_ws/results/experiments/openloop/laptop/orb2/")
Result_root = "/tmp/orb2"

# Number_GF_List = [400, 800, 1000, 1500]
Number_GF_List = [1200]  # , 200]  # , 400]
NumRepeating = 10  # 10 # 20 #  5 #
SpeedPool = [1.0, 2.0, 3.0, 4.0, 5.0]  # , 3.0]  # x
SleepTime = 1  # 10 # 25
EnableViewer = False
EnableLogging = 1

ORB2_PATH = os.path.join(os.environ["HOME"], "closedloop_ws/src/ORB_SLAM2")
ConfigPath = os.path.join(ORB2_PATH, "Examples/Stereo")


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
                file_setting = os.path.join(ConfigPath, "EuRoC.yaml")

                file_vocab = os.path.join(ORB2_PATH, "Vocabulary/ORBvoc.txt")

                file_traj = os.path.join(Experiment_dir, SeqName)
                file_dummy_map = file_traj + "_dummy_map.txt"
                file_log = "> " + file_traj + "_logging.txt" if EnableLogging else ""
                file_data = os.path.join(DATA_ROOT, SeqName)
                file_rosbag = os.path.join(DATA_ROOT, SeqName + ".bag")
                # file_timestamp = os.path.join(file_data, "times.txt")

                # compose cmd
                cmd_slam = str(
                    "rosrun ORB_SLAM2 Stereo "
                    + file_vocab
                    + " "
                    + file_setting
                    + " "
                    + str(int(num_gf))
                    + " "
                    + "false"
                    + " "
                    + str(1 if EnableViewer else 0)
                    + " "
                    + "/cam0/image_raw /cam1/image_raw"
                    + " "
                    + file_traj
                    + " "
                    + file_log
                )

                cmd_rosbag = "rosbag play " + file_rosbag + " -r " + str(speed)  # + ' -u 20'

                print(bcolors.WARNING + "cmd_slam: \n" + cmd_slam + bcolors.ENDC)

                print(bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC)
                subprocess.Popen(cmd_slam, shell=True)

                print(bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC)
                time.sleep(SleepTime * 3)

                print(bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC)
                proc_bag = subprocess.call(cmd_rosbag, shell=True)

                print(bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC)
                subprocess.call("rosnode kill Stereo", shell=True)
                time.sleep(SleepTime)
                # print bcolors.OKGREEN + "Saving the map to file system" + bcolors.ENDC
                # time.sleep(15)
                subprocess.call("pkill Stereo", shell=True)
