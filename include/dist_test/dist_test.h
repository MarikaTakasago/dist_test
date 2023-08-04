#ifndef DIST_TEST_H
#define DIST_TEST_H

//ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

//opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class DistTest
{
    public:
        DistTest();

    private:
        //methods
        void image_callback(const sensor_msgs::ImageConstPtr& msg);
        void measure_distance(cv::Mat& img,int x,int y);
        void linear_approximation(double x1,double y1,double x2,double y2,double& a,double& b);
        void display_distance(cv::Mat img,double dist);

        //variables
        bool debug_;

        //dist [m]
        int target_x;
        int target_y;
        double target_dist;
        int ref1_x_;//LiDAR
        int ref1_y_;
        double ref1_dist_;
        int ref2_x_;//MiSUMi or pocket
        int ref2_y_;
        double ref2_dist_;

        //ros
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber sub_image_;
        ros::Publisher pub_dist_;

        //for debug
        ros::Subscriber sub_theta_;
        ros::Publisher pub_image_;
        void theta_callback(const sensor_msgs::ImageConstPtr& msg);

};

#endif // DIST_TEST_H
