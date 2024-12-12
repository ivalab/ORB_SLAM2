/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/publisher.h>

#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;


    ros::Publisher mpCameraPosePublisher, mpCameraPoseInIMUPublisher;
    ros::Publisher mpFrameWithInfoPublisher;

    std::vector<double> vTimesTrack;
    std::vector<pair<double, double> > vStampedTimesTrack;

    void saveStats(const std::string& path_traj)
    {
        // Tracking time statistics
        sort(vTimesTrack.begin(),vTimesTrack.end());
        float totaltime = 0;
        int proccIm = vTimesTrack.size();
        for(int ni=0; ni<proccIm; ni++)
        {
            totaltime+=vTimesTrack[ni];
        }

        // save to stats
        {
            std::ofstream myfile(path_traj + "_stats.txt");
            myfile << "#dummy processed_img_num mean_time median_time min_time max_time\n";
            myfile << std::setprecision(6) << -1 << " "
                << proccIm << " "
                << totaltime / proccIm << " "
                << vTimesTrack[proccIm/2] << " "
                << vTimesTrack.front() << " "
                << vTimesTrack.back() << " ";
            myfile.close();

            myfile.open(path_traj + "_Log_Latency.txt");
            myfile << "#timestamp tracking_time\n";
            myfile << std::setprecision(20);
            for (const auto& m : vStampedTimesTrack)
            {
                myfile << m.first << " " << m.second << "\n";
            }
            myfile.close();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc < 8)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings feature_num do_rectify do_vis cam0_topic cam1_topic path_to_traj" << endl;
        ros::shutdown();
        return 1;
    }    

    const bool do_viz = std::stoi(argv[5]);
    std::cout << "viz: " << do_viz << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,do_viz);
    SLAM.mpTracker->updateORBExtractor(stoi(argv[3]));
    // realtime trajectory logging
    std::string fNameRealTimeTrack = std::string(argv[8]) + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    SLAM.mpTracker->SetRealTimeFileStream(fNameRealTimeTrack);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[4]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, argv[6], 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, argv[7], 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    igb.mpCameraPosePublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ORB_SLAM/camera_pose", 100);
    igb.mpCameraPoseInIMUPublisher = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ORB_SLAM/camera_pose_in_imu", 100);
    igb.mpFrameWithInfoPublisher = nh.advertise<sensor_msgs::Image>("ORB_SLAM/frame_with_info", 100);

    while (ros::ok())
    {
        ros::spin();
    }

    cout << "ros_stereo: done with spin!" << endl;

    // save stats
    igb.saveStats(std::string(argv[8]));

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(std::string(argv[8]) + "_KeyFrameTrajectory.txt");
    SLAM.SaveTrackingLog(std::string(argv[8]) + "_Log.txt" );
    SLAM.SaveMappingLog(std::string(argv[8]) + "_Log_Mapping.txt");

    std::cout << "Finished saving!" << std::endl;

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();
    cout << "ros_stereo: done with SLAM Shutdown!" << endl;

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
#ifdef ENABLE_CLOSED_LOOP
    // @NOTE (yanwei) throw the first few garbage images from gazebo
    static size_t skip_imgs = 0;
    if (skip_imgs < 10)
    {
        ++skip_imgs;
        return;
    }
#endif
    const double latency_trans = ros::Time::now().toSec() - msgLeft->header.stamp.toSec();
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat pose;
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        pose = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        pose = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }


    // collect latency
    double latency_total = ros::Time::now().toSec() - cv_ptrLeft->header.stamp.toSec();
    {
        const double track_latency = latency_total - latency_trans;
        vTimesTrack.emplace_back(track_latency);
        vStampedTimesTrack.emplace_back(cv_ptrLeft->header.stamp.toSec(), track_latency);
    }

    if (pose.empty())
    {
        return;
    }

    cv::Mat Rwc = pose.rowRange(0,3).colRange(0,3).t();
    cv::Mat twc = -Rwc*pose.rowRange(0,3).col(3);
    tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
		    Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
		    Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
    tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

    tf::Transform tfTcw(M,V);
    geometry_msgs::Transform gmTwc;
    tf::transformTFToMsg(tfTcw, gmTwc);
	
    geometry_msgs::Pose camera_pose;
    camera_pose.position.x = gmTwc.translation.x;
    camera_pose.position.y = gmTwc.translation.y;
    camera_pose.position.z = gmTwc.translation.z;
    camera_pose.orientation = gmTwc.rotation;
    
    geometry_msgs::PoseWithCovarianceStamped camera_odom;
    camera_odom.header.frame_id = "odom";
    camera_odom.header.stamp = cv_ptrLeft->header.stamp;
    camera_odom.pose.pose = camera_pose;
    
    mpCameraPosePublisher.publish(camera_odom);

//
// by default, an additional transform is applied to make camera pose and body frame aligned
// which is assumed in msf
#ifdef INIT_WITH_ARUCHO
    tf::Matrix3x3 Ric(   0, -1, 0,
			    0, 0, -1,
			    1, 0, 0);
/*  tf::Matrix3x3 Ric(   0, 0, 1,
			    -1, 0, 0,
			    0, -1, 0);*/
	tf::Transform tfTiw ( tf::Matrix3x3( tfTcw.getRotation() ) * Ric, tfTcw.getOrigin() );
#else
    tf::Matrix3x3 Ric( 0,  0,  1,
			-1,  0,  0,
			0,  -1,  0);
      tf::Transform tfTiw ( Ric * tf::Matrix3x3( tfTcw.getRotation() ), Ric * tfTcw.getOrigin() );
#endif

    geometry_msgs::Transform gmTwi;
	tf::transformTFToMsg(tfTiw, gmTwi);
	
	geometry_msgs::Pose camera_pose_in_imu;
	camera_pose_in_imu.position.x = gmTwi.translation.x;
	camera_pose_in_imu.position.y = gmTwi.translation.y;
	camera_pose_in_imu.position.z = gmTwi.translation.z;
	camera_pose_in_imu.orientation = gmTwi.rotation;
    
    geometry_msgs::PoseWithCovarianceStamped camera_odom_in_imu;
	camera_odom_in_imu.header.frame_id = "map";
	camera_odom_in_imu.header.stamp = cv_ptrLeft->header.stamp;
	camera_odom_in_imu.pose.pose = camera_pose_in_imu;
    
	mpCameraPoseInIMUPublisher.publish(camera_odom_in_imu);
    
#ifdef FRAME_WITH_INFO_PUBLISH
    if (mpSLAM != NULL && mpSLAM->mpFrameDrawer != NULL) {
        cv::Mat fr_info_cv = mpSLAM->mpFrameDrawer->DrawFrame();
        cv_bridge::CvImage out_msg;
        out_msg.header   = cv_ptrLeft->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
        out_msg.image    = fr_info_cv; // Your cv::Mat
        mpFrameWithInfoPublisher.publish(out_msg.toImageMsg());
    }
#endif

}


