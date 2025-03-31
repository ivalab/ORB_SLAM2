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

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;

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

    // if(argc != 3)
    if (argc < 7)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings num_all_feature do_vis cam0_topic depth_topic path_to_traj" << endl;        
        ros::shutdown();
        return 1;
    }    

    const bool do_viz = std::stoi(argv[4]);
    std::cout << "viz: " << do_viz << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,do_viz);
    SLAM.mpTracker->updateORBExtractor(stoi(argv[3]));

    // realtime trajectory logging
    std::string fNameRealTimeTrack = std::string(argv[7]) + "_AllFrameTrajectory.txt";
    std::cout << std::endl << "Saving AllFrame Trajectory to AllFrameTrajectory.txt" << std::endl;
    SLAM.mpTracker->SetRealTimeFileStream(fNameRealTimeTrack);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, argv[5], 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, argv[6], 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::spin();

    igb.saveStats(std::string(argv[7]));

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(std::string(argv[7]) + "_KeyFrameTrajectory.txt");
    SLAM.SaveTrackingLog(std::string(argv[7]) + "_Log.txt" );
    SLAM.SaveMappingLog(std::string(argv[7]) + "_Log_Mapping.txt");

    std::cout << "Finished saving!" << std::endl;

    ros::shutdown();

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    const double latency_trans = ros::Time::now().toSec() - msgRGB->header.stamp.toSec();

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    // collect latency
    double latency_total = ros::Time::now().toSec() - cv_ptrRGB->header.stamp.toSec();
    {
        const double track_latency = latency_total - latency_trans;
        vTimesTrack.emplace_back(track_latency);
        vStampedTimesTrack.emplace_back(cv_ptrRGB->header.stamp.toSec(), track_latency);
    }

    if (mpSLAM->mpTracker->mState != 2) // OK
    {
        return;
    }

}