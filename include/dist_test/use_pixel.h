#ifndef WITH_LIDAR_H
#define WITH_LIDAR_H

#include "geometry_msgs/PoseStamped.h"
#include <opencv2/core/mat.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
// #include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf/tf.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vector>
#include <math.h>
#include <string.h>
#include <memory.h>
#include <utility>
#include <algorithm>

#include <camera_apps_msgs/BoundingBox.h>
#include <camera_apps_msgs/BoundingBoxes.h>
#include <camera_apps_msgs/Mask.h>
#include <camera_apps_msgs/Masks.h>

class WithLidar
{
    public:
        WithLidar();

    private:
        struct Param
        {
            int referenced_laser_num;
            float offset_angle_camera_to_lidar;


            std::string scan_topic_name;
            std::string masks_topic_name;
        };

        void synchro_callback(const sensor_msgs::LaserScanConstPtr& scan_msg,
                              const camera_apps_msgs::MasksConstPtr& masks_msg);

        static std::vector<int> calc_most_masked_col_idxs(const cv::Mat& mask, int referenced_laser_num);
        static std::vector<float> col_idxs_to_angles(const std::vector<int>& col_idxs, int img_width);
        static float convert_angle_camera_to_lidar(float angle_at_camera, float offset_angle_camera_to_lidar);
        static float adjust_angle(float angle);
        static std::optional<geometry_msgs::Pose> calc_pose_from_angles(
                const sensor_msgs::LaserScan& scan, const std::vector<float>& angles);
        static std::optional<std::pair<float, float>> calc_point_from_angle(
                const sensor_msgs::LaserScan& scan, float angle);
        static std::optional<cv::Mat> convert_img_msg_to_cv(const sensor_msgs::Image& img_msg);

        Param param_;

        //msg
        // sensor_msgs::LaserScan scan_;
        // camera_apps_msgs::Masks masks_;
        
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
                camera_apps_msgs::Masks> MySyncPolicy;
        // message_filter関連のクラスはインスタンス化するとコピーできないのでポインタとして宣言
        std::unique_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> scan_sub_ptr_;
        std::unique_ptr<message_filters::Subscriber<camera_apps_msgs::Masks>> masks_sub_ptr_;
        std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> synchronizer_ptr_;

        ros::Publisher person_poses_pub_;
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
};
#endif
