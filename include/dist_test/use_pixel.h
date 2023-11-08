#ifndef WITH_LIDAR_H
#define WITH_LIDAR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf/tf.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

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
        ~WithLidar();

    private:
        struct Param
        {
            int scan_line_sum;
            int scan_angle;
            int laser_num;
            bool display;
            float conf_th;
            float lower;
            float upper;

            std::string detected_image_topic_name;
            std::string scan_topic_name;
            std::string masks_topic_name;
        };

        void image_callback(const sensor_msgs::Image::ConstPtr& msg);
        void synchro_callback(const sensor_msgs::LaserScanConstPtr &scan_msg,
                const camera_apps_msgs::MasksConstPtr& masks_msg);

        void measure_danda(); // danda = "d"istance "and" "a"ngle
        void calculate_id_pic(int bboxmin,int size);
        void calculate_id_box(int xmin,int xmax);
        void display_distances(int person_num);
        void get_masked_pixels(cv::Mat img);

        Param param_;
        //valiables
        cv::Mat input_image_;
        int image_width_;
        int image_height_;
        int image_center_x_;

        int scan_hight_;
        int pic_id_max_;
        int pic_id_min_;
        std::vector<double> pic_id_;
        int box_id_max_;
        int box_id_min_;


        // double distance_;
        // double angle_;
        std::vector<double> distance_;
        std::vector<double> angle_;

        double a_;
        double b_;

        //mask image
        cv::Mat mask_image_;
        std::vector<int> masked_pixels_;

        //checker
        bool get_image_ = false;
        bool get_scan_ = false;
        bool get_bbox_ = false;

        //ros
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber sub_image_;
        ros::Publisher pub_poses_;
        ros::Publisher pub_image_; //for debug
        
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
                camera_apps_msgs::Masks> MySyncPolicy;
        message_filters::Subscriber<sensor_msgs::LaserScan> *scan_sub_;
        message_filters::Subscriber<camera_apps_msgs::Masks> *masks_sub_;
        message_filters::Synchronizer<MySyncPolicy> *synchro_;

        //msg
        sensor_msgs::LaserScan scan_;
        sensor_msgs::Image image_msg_;
        sensor_msgs::Image mask_image_msg;
        camera_apps_msgs::Masks masks_;
        camera_apps_msgs::BoundingBoxes bboxes_;
        geometry_msgs::PoseArray person_poses_;
};
#endif

