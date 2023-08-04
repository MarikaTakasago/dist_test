#ifndef MF_WITH_LIDAR_H
#define MF_WITH_LIDAR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <geometry_mssg/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <tf2_ros/utils.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <math.h>
#include <vector>
#include <string>

#include "camera_apps_msgs/BoundingBox.h"
#include "camera_apps_msgs/BoundingBoxes.h"

//function
void calc_pose(const sensor_msgs::LaserScan::ConstPtr& scan, const camera_apps_msgs::BoundingBoxes::ConstPtr& boxes, int person_num, int image_width);
void calc_id(int xmin, int xmax, int &id_min, int &id_max, double &angle);
void display_poses(int num, const sensor_msgs::Image::ConstPtr& msg);
void get_tf();

//valuables
double diff_x_;
double diff_y_;
double diff_yaw_;

double conf_th_;

//topics
std::string scan_topic_;
std::string image_topic_;
std::string bounding_box_topic_;
std::string pose_array_topic_;
std::string person_frame_;

//ros
ros::publisher pub_image_;
ros::publisher pub_poses_;

//tf
tf2_ros::Buffer tf_buffer_;
tf2_ros::TransformListener tf_listener_;


#endif  // MF_WITH_LIDAR_H
