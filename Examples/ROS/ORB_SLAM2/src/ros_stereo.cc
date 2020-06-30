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
#include <torch/torch.h>

#include"../../../include/System.h"
//#include "yolo.h"
using namespace std;
torch::DeviceType device_type;
torch::Device device(torch::kCUDA);
ORB_SLAM2::Darknet net("/home/wcx/libtorch-yolov3/models/yolov3.cfg", &device);
int input_image_size = 416;
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
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

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/zed/zed_node/left/image_rect_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/zed/zed_node/right/image_rect_color", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);

    //wcx


   //ORB_SLAM2::Darknet net("/home/wcx/libtorch-yolov3/models/yolov3.cfg", &device);

    map<string, string> *info = net.get_net_info();

    info->operator[]("height") = std::to_string(input_image_size);

    std::cout << "loading weight ..." << endl;
    net.load_weights("/home/wcx/libtorch-yolov3/models/yolov3.weights");
    std::cout << "weight loaded ..." << endl;

    net.to(device);

    torch::NoGradGuard no_grad;
    net.eval();

    std::cout << "start to inference ..." << endl;


    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    //wcx


/*    if (torch::cuda::is_available() ) {
        device_type = torch::kCUDA;
    } else {
        device_type = torch::kCPU;
    }
    torch::Device device(device_type);*/

    // input image size for YOLO v3


    cv::Mat origin_image, resized_image;
    //wcx

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

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);

        //torch --wcx
        vector<ORB_SLAM2::My_label> imbbx;
        origin_image = imLeft;

        cv::cvtColor(origin_image, resized_image,  cv::COLOR_RGB2BGR);
        cv::resize(resized_image, resized_image, cv::Size(input_image_size, input_image_size));

        cv::Mat img_float;
        resized_image.convertTo(img_float, CV_32F, 1.0/255);

        auto img_tensor = torch::from_blob(img_float.data, {1, input_image_size, input_image_size, 3}).to(device);
        img_tensor = img_tensor.permute({0,3,1,2});

     //   auto start = std::chrono::high_resolution_clock::now();

        auto output = net.forward(img_tensor);

        // filter result by NMS
        // class_num = 80
        // confidence = 0.6
        auto result = net.write_results(output, 80, 0.6, 0.4);

     //   auto end = std::chrono::high_resolution_clock::now();

    //    auto duration = duration_cast<milliseconds>(end - start);

        if(result.dim() >1)
        {
            int obj_num = result.size(0);

            float w_scale = float(origin_image.cols) / input_image_size;
            float h_scale = float(origin_image.rows) / input_image_size;

            result.select(1,1).mul_(w_scale);
            result.select(1,2).mul_(h_scale);
            result.select(1,3).mul_(w_scale);
            result.select(1,4).mul_(h_scale);

            auto result_data = result.accessor<float, 2>();

            for (int i = 0; i < result.size(0) ; i++)
            {
                ORB_SLAM2::My_label label;
                //cv::rectangle(origin_image, cv::Point(result_data[i][1], result_data[i][2]), cv::Point(result_data[i][3], result_data[i][4]), cv::Scalar(0, 0, 255), 1, 1, 0);
                label.label_class = result_data[i][7];
                label.position[0] = (result_data[i][1] + result_data[i][3])/2;
                label.position[1] = (result_data[i][2] + result_data[i][4])/2;
                label.position[2] = abs(result_data[i][3]-result_data[i][1]);
                label.position[3] = abs(result_data[i][4]-result_data[i][2]);
                label.confidence =  result_data[i][6];
                imbbx.push_back(label);
            }

        }
        //wcx

        mpSLAM->TrackStereo_bbx(imLeft,imRight,cv_ptrLeft->header.stamp.toSec(),imbbx);

       // mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        //torch --wcx
        vector<ORB_SLAM2::My_label> imbbx;
        origin_image = cv_ptrLeft->image;

        cv::cvtColor(origin_image, resized_image,  cv::COLOR_RGB2BGR);
        cv::resize(resized_image, resized_image, cv::Size(input_image_size, input_image_size));

        cv::Mat img_float;
        resized_image.convertTo(img_float, CV_32F, 1.0/255);

        auto img_tensor = torch::from_blob(img_float.data, {1, input_image_size, input_image_size, 3}).to(device);
        img_tensor = img_tensor.permute({0,3,1,2});

        //   auto start = std::chrono::high_resolution_clock::now();

        auto output = net.forward(img_tensor);

        // filter result by NMS
        // class_num = 80
        // confidence = 0.6
        auto result = net.write_results(output, 80, 0.6, 0.4);

        //   auto end = std::chrono::high_resolution_clock::now();

        //    auto duration = duration_cast<milliseconds>(end - start);

        if(result.dim() >1)
        {
            int obj_num = result.size(0);

            float w_scale = float(origin_image.cols) / input_image_size;
            float h_scale = float(origin_image.rows) / input_image_size;

            result.select(1,1).mul_(w_scale);
            result.select(1,2).mul_(h_scale);
            result.select(1,3).mul_(w_scale);
            result.select(1,4).mul_(h_scale);

            auto result_data = result.accessor<float, 2>();

            for (int i = 0; i < result.size(0) ; i++)
            {
                ORB_SLAM2::My_label label;
                //cv::rectangle(origin_image, cv::Point(result_data[i][1], result_data[i][2]), cv::Point(result_data[i][3], result_data[i][4]), cv::Scalar(0, 0, 255), 1, 1, 0);
                label.label_class = result_data[i][7];
                label.position[0] = (result_data[i][1] + result_data[i][3])/2;
                label.position[1] = (result_data[i][2] + result_data[i][4])/2;
                label.position[2] = abs(result_data[i][3]-result_data[i][1]);
                label.position[3] = abs(result_data[i][4]-result_data[i][2]);
                label.confidence =  result_data[i][6];
                imbbx.push_back(label);
            }

        }
        //wcx
      //  cout<<"imbbx的size"<<imbbx.size()<<endl;
        mpSLAM->TrackStereo_bbx(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec(),imbbx);
    }

}


