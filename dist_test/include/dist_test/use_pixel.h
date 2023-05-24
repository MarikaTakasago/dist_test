#ifndef WITH_LIDAR_H
#define WITH_LIDAR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf/tf.h>

#include <math.h>
#include <string.h>

#include "camera_apps_msgs/BoundingBox.h"
#include "camera_apps_msgs/BoundingBoxes.h"
#include "camera_apps_msgs/Mask.h"
#include "camera_apps_msgs/Masks.h"

class WithLidar
{
    public:
        WithLidar();

    private:
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void image_callback(const sensor_msgs::Image::ConstPtr& msg);
        void bbox_callback(const camera_apps_msgs::Masks::ConstPtr& msg);

        void measure_danda(); // danda = "d"istance "and" "a"ngle
        void calculate_id_pic(int bboxmin,int size);
        void calculate_id_box(int xmin,int xmax);
        void display_distances(int person_num);
        void get_masked_pixels(cv::Mat img);

        //valiables
        cv::Mat input_image_;
        int image_width_;
        int image_height_;
        int image_center_x_;

        int scan_hight_;
        int scan_line_sum_;
        int scan_angle_;
        int pic_id_max_;
        int pic_id_min_;
        std::vector<double> pic_id_;
        int box_id_max_;
        int box_id_min_;
        int laser_num_;

        double conf_th_;

        // double distance_;
        // double angle_;
        std::vector<double> distance_;
        std::vector<double> angle_;

        double a_;
        double b_;

        //mask image
        cv::Mat mask_image_;
        double lower_ = 0.1;
        double upper_ = 0.3;
        std::vector<int> masked_pixels_;

        //checker
        bool get_image_ = false;
        bool get_scan_ = false;
        bool get_bbox_ = false;
        bool display_ = false;

        //ros
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber sub_scan_;
        ros::Subscriber sub_image_;
        ros::Subscriber sub_bbox_;
        ros::Publisher pub_poses_;
        ros::Publisher pub_image_; //for debug

        //msg
        sensor_msgs::LaserScan scan_;
        sensor_msgs::Image image_msg_;
        sensor_msgs::Image mask_image_msg;
        camera_apps_msgs::Masks masks_;
        camera_apps_msgs::BoundingBoxes bboxes_;
        geometry_msgs::PoseArray person_poses_;
};
#endif

