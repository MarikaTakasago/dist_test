#ifndef WITH_LIDAR_MF_H
#define WITH_LIDAR_MF_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <math.h>

#include "camera_apps_msgs/BoundingBox.h"
#include "camera_apps_msgs/BoundingBoxes.h"
#include "camera_apps_msgs/Mask.h"
#include "camera_apps_msgs/Masks.h"


void measure_danda(); // danda = "d"istance "and" "a"ngle
void calculate_id(int xmin,int xmax);
void display_distances(int person_num);

//valiables
cv::Mat input_image_;
int image_width_;
int image_center_x_;

int scan_line_sum_;
int scan_angle_;
int id_max_;
int id_min_;
std::vector<double> id_;
int laser_num_;

double conf_th_;

// double distance_;
// double angle_;
std::vector<double> distance_;
std::vector<double> angle_;


//ros
ros::Publisher pub_poses_;

//msg
sensor_msgs::LaserScan scan_;
sensor_msgs::Image image_msg_;
camera_apps_msgs::Masks masks_;
camera_apps_msgs::BoundingBoxes bboxes_;
geometry_msgs::PoseArray person_poses_;

#endif // WITH_LIDAR_MF_H
