#include "dist_test/with_lidar_mf.h"

void sync_callback(const sensor_msgs::LaserScan::ConstPtr& lidar_msg,
                   const camera_apps_msgs::Masks::ConstPtr& masks_msg)
{
    ROS_INFO("received msgs");
    //scan
    scan_ = *lidar_msg;
    scan_line_sum_ = scan_.ranges.size() -1;

    //masks
    masks_ = *masks_msg;
    if(masks_.masks.size() == 0)
    {
        std::cout<<"no object"<<std::endl;
        return;
    }
    //reset vector and counter
    int person_num = 0;
    distance_.clear();
    angle_.clear();
    id_.clear();
    person_poses_.poses.clear();

    for(auto mask : masks_.masks)
    {
        //get bbox
        camera_apps_msgs::BoundingBox bbox = mask.bounding_box;
        //if label is not person skip
        if(bbox.label != "person" || bbox.confidence < conf_th_) continue;
        int xmin = bbox.xmin;
        int ymin = bbox.ymin;
        int xmax = bbox.xmax;
        int ymax = bbox.ymax;

        //if person is not in the scan angle : skip
        if(xmax < image_width_/8 || image_width_*7/8 < xmin) continue;
        if(xmin < image_width_/8) xmin = image_width_/8;
        if(image_width_*7/8 < xmax) xmax = image_width_*7/8;

        // calculate_id(xmin,xmax);
        //calc id from grid x
        double lidar_fi_min = M_PI*xmin*2.0/image_width_;
        double lidar_fi_max = M_PI*xmax*2.0/image_width_;
        // double fi_min = lidar_fi_min + M_PI/4.0;
        // double fi_max = lidar_fi_max + M_PI/4.0;
        // if(fi_min > 2*M_PI) fi_min -= 2*M_PI;
        // if(fi_max > 2*M_PI) fi_max -= 2*M_PI;
        std::cout << "fi_min : " << (lidar_fi_min) * 180/M_PI << std::endl;
        std::cout << "fi_max : " << (lidar_fi_max) * 180/M_PI << std::endl;
        // std::cout << "id_min : " << id_min_ << std::endl;
        // std::cout << "id_max : " << id_max_ << std::endl;
        double lidar_fi_ave = (lidar_fi_min + lidar_fi_max)/2.0;
        double fi_ave = lidar_fi_ave + M_PI/4.0;
        if(fi_ave > 2*M_PI) fi_ave -= 2*M_PI;
        if(lidar_fi_ave > 2*M_PI) lidar_fi_ave -= 2*M_PI;
        if(lidar_fi_min > 2*M_PI) lidar_fi_min -= 2*M_PI;
        if(lidar_fi_max > 2*M_PI) lidar_fi_max -= 2*M_PI;
        //0~360 -> -180 ~ 180
        if(fi_ave > M_PI) fi_ave -= 2*M_PI;
        // if(lidar_fi_ave > M_PI) lidar_fi_ave -= 2*M_PI;
        double angle = lidar_fi_ave - M_PI - M_PI/4.0;
        double angle_min = lidar_fi_min - M_PI - M_PI/4.0;
        double angle_max = lidar_fi_max - M_PI - M_PI/4.0;
        angle_.push_back(angle);
        id_max_ = -8*angle_min*180/M_PI + 720;
        id_min_ = -8*angle_max*180/M_PI + 720;
        std::cout << "id_min : " << id_min_ << std::endl;
        std::cout << "id_max : " << id_max_ << std::endl;
        //end calc_id

        // measure_danda(); //calculate distance and angle
        std::vector<double> scan_data;
        scan_data.clear();
        for(int i=id_min_;i<=id_max_;i++)
        {
            if(scan_.ranges[i] > scan_.range_min && scan_.ranges[i] < scan_.range_max)
            {
                scan_data.push_back(scan_.ranges[i]);
            }
        }
        if(scan_data.size() == 0) return;
        std::cout << "center : " << scan_.ranges[(id_min_+id_max_)/2] << std::endl;
        //sort min to max
        for(int i=0;i<scan_data.size()-1;i++)
        {
            for(int j=i+1;j<scan_data.size();j++)
            {
                if(scan_data[i] > scan_data[j])
                {
                    double tmp = scan_data[i];
                    scan_data[i] = scan_data[j];
                    scan_data[j] = tmp;
                }
            }
        }
        // std::sort(scan_data.begin(),scan_data.end());
        //calc average of minimum 10 points
        double sum = 0;
        int count = 0;
        int lim = laser_num_;
        if(scan_data.size() < laser_num_) lim = scan_data.size();
        for(int i = 0; i < lim; i++)
        {
            std::cout << scan_data[i] <<" " ;
            sum += scan_data[i];
            count++;
        }
        std::cout << std::endl;
        double dist = sum / count;
        // std::cout << "dist : " << sum << "/" << (laser_num_-skip) << "=" << dist << std::endl;
        distance_.push_back(dist);

        //calc angle from id(center)
        double center_id = (id_min_ + id_max_) / 2.0;
        id_.push_back(center_id);
        double center_angle = center_id * scan_angle_ / scan_line_sum_ - 180;
        //if id = 0, angle = -180
        // angle = angle * M_PI / 180; //to radian
        // angle_.push_back(angle);
        // end danda

        //for debug
        std::cout << "x range : " << xmin << "," << xmax<< std::endl;
        std::cout << "scan_id : " << (id_min_ + id_max_)/2.0 << std::endl;
        std::cout << "distance : " << distance_[person_num] << std::endl;
        std::cout << "angle : " << angle_[person_num]*180/M_PI << std::endl;
        std::cout<<"==========================================="<<std::endl;

        //to msg
        geometry_msgs::Pose pose;
        pose.position.x = distance_[person_num]*cos(angle_[person_num]) - 0.07;
        pose.position.y = -distance_[person_num]*sin(angle_[person_num]) - 0.07;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 0;
        person_poses_.poses.push_back(pose);

        person_num++;
    }
    //pub poses
    pub_poses_.publish(person_poses_);

}



void calculate_id(int xmin,int xmax)
{
    //calc id from grid x
    double lidar_fi_min = M_PI*xmin*2.0/image_width_;
    double lidar_fi_max = M_PI*xmax*2.0/image_width_;
    id_min_ = (scan_line_sum_*2/(3*M_PI))*lidar_fi_min - scan_line_sum_/6;
    id_max_ = (scan_line_sum_*2/(3*M_PI))*lidar_fi_max - scan_line_sum_/6;
    // double fi_min = lidar_fi_min + M_PI/4.0;
    // double fi_max = lidar_fi_max + M_PI/4.0;
    // if(fi_min > 2*M_PI) fi_min -= 2*M_PI;
    // if(fi_max > 2*M_PI) fi_max -= 2*M_PI;
    std::cout << "fi_min : " << (lidar_fi_min) * 180/M_PI << std::endl;
    std::cout << "fi_max : " << (lidar_fi_max) * 180/M_PI << std::endl;
    // std::cout << "id_min : " << id_min_ << std::endl;
    // std::cout << "id_max : " << id_max_ << std::endl;
    double lidar_fi_ave = (lidar_fi_min + lidar_fi_max)/2.0;
    double fi_ave = lidar_fi_ave + M_PI/4.0;
    if(fi_ave > 2*M_PI) fi_ave -= 2*M_PI;
    if(lidar_fi_ave > 2*M_PI) lidar_fi_ave -= 2*M_PI;
    if(lidar_fi_min > 2*M_PI) lidar_fi_min -= 2*M_PI;
    if(lidar_fi_max > 2*M_PI) lidar_fi_max -= 2*M_PI;
    //0~360 -> -180 ~ 180
    if(fi_ave > M_PI) fi_ave -= 2*M_PI;
    // if(lidar_fi_ave > M_PI) lidar_fi_ave -= 2*M_PI;
    double angle = lidar_fi_ave - M_PI - M_PI/4.0;
    double angle_min = lidar_fi_min - M_PI - M_PI/4.0;
    double angle_max = lidar_fi_max - M_PI - M_PI/4.0;
    angle_.push_back(angle);
    // id_min_ = 8*angle_min + 1440;
    // id_max_ = 8*angle_max + 1440;
    std::cout << "id_min : " << id_min_ << std::endl;
    std::cout << "id_max : " << id_max_ << std::endl;

}

void measure_danda()
{
    //danda means distance and angle
    //calc distance from id
    //use average of minimun 10 points
    //sort lidar data min to max
    std::vector<double> scan_data;
    scan_data.clear();
    for(int i=id_min_;i<=id_max_;i++)
    {
        if(scan_.ranges[i] > scan_.range_min && scan_.ranges[i] < scan_.range_max)
        {
            scan_data.push_back(scan_.ranges[i]);
        }
    }
    if(scan_data.size() == 0) return;
    std::cout << "center : " << scan_.ranges[(id_min_+id_max_)/2] << std::endl;
    //sort min to max
    for(int i=0;i<scan_data.size()-1;i++)
    {
        for(int j=i+1;j<scan_data.size();j++)
        {
            if(scan_data[i] > scan_data[j])
            {
                double tmp = scan_data[i];
                scan_data[i] = scan_data[j];
                scan_data[j] = tmp;
            }
        }
    }
    // std::sort(scan_data.begin(),scan_data.end());
    //calc average of minimum 10 points
    double sum = 0;
    int count = 0;
    int lim = laser_num_;
    if(scan_data.size() < laser_num_) lim = scan_data.size();
    for(int i = 0; i < lim; i++)
    {
        std::cout << scan_data[i] <<" " ;
        sum += scan_data[i];
        count++;
    }
    std::cout << std::endl;
    double dist = sum / count;
    // std::cout << "dist : " << sum << "/" << (laser_num_-skip) << "=" << dist << std::endl;
    distance_.push_back(dist);

    //calc angle from id(center)
    double center_id = (id_min_ + id_max_) / 2.0;
    id_.push_back(center_id);
    double angle = center_id * scan_angle_ / scan_line_sum_ - 180; //if id = 0, angle = -180
    // angle = angle * M_PI / 180; //to radian
    // angle_.push_back(angle);
}


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, camera_apps_msgs::Masks> MySyncPolicy;

int main(int argc , char **argv)
{
    ros::init(argc, argv, "with_lidar_mf");
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_("~");

    //get parameters
    private_nh_.param("scan_line_sum", scan_line_sum_, 2161);
    private_nh_.param("scan_angle", scan_angle_, 270);
    private_nh_.param("laser_num", laser_num_, 10);
    private_nh_.param("conf_th_", conf_th_, 0.5);
    private_nh_.param("image_width", image_width_, 1280);

    pub_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/person_poses",1);

    id_min_ = id_max_ = 0;
    person_poses_.header.frame_id = "base_link";
    person_poses_.poses.resize(0);

    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan(nh_, "/scan", 1);
    message_filters::Subscriber<camera_apps_msgs::Masks> sub_masks(nh_, "/masks", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), sub_scan, sub_masks);
    sync.registerCallback(&sync_callback);

    ROS_INFO("with_lidar_mf node is ready");
    ros::spin();
    return 0;
}
