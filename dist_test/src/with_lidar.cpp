#include <dist_test/with_lidar.h>

WithLidar::WithLidar() : private_nh_("~")
{
    private_nh_.param("scan_line_sum", scan_line_sum_, 2161);
    private_nh_.param("scan_angle", scan_angle_, 270);
    private_nh_.param("laser_num", laser_num_, 10);
    private_nh_.param("display", display_, false);
    private_nh_.param("conf_th_", conf_th_, 0.5);

    sub_scan_ = nh_.subscribe("/scan",1,&WithLidar::scan_callback,this);
    sub_image_ = nh_.subscribe("/detected_image",1,&WithLidar::image_callback,this);
    sub_bbox_ = nh_.subscribe("/masks",1,&WithLidar::bbox_callback,this);

    pub_image_ = nh_.advertise<sensor_msgs::Image>("/with_lidar",1);
    pub_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/person_poses",1);

    id_min_ = id_max_ = 0;
    person_poses_.header.frame_id = "base_link";
    person_poses_.poses.resize(0);
}

void WithLidar::scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_ = *msg;
    get_scan_ = true;
    //check laser num
    scan_line_sum_ = scan_.ranges.size() - 1;
}
#include <string.h>

void WithLidar::image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    image_msg_ = *msg;
    image_center_x_ = image_msg_.width/2;
    image_width_ = image_msg_.width;
    // ROS_INFO("receive image");
    //to cv image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_msg_,sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    input_image_ = cv_ptr->image;
    get_image_ = true;
}

void WithLidar::bbox_callback(const camera_apps_msgs::Masks::ConstPtr& msg)
{
    masks_ = *msg;
    ROS_INFO("receive bbox");
    if(masks_.masks.size() == 0)
    {
        std::cout<<"no person"<<std::endl;
        return;
    }

    if(get_image_ && get_scan_)
    {
        get_bbox_ = true;

        //reset vector and counter
        int person_num = 0;
        distance_.clear();
        angle_.clear();
        id_.clear();
        person_poses_.poses.clear();
        person_poses_.header.stamp = scan_.header.stamp;

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

            calculate_id(xmin,xmax);
            measure_danda(); //calculate distance and angle

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
        if(display_) display_distances(person_num);
        //pub poses
        pub_poses_.publish(person_poses_);

    }
}

void WithLidar::calculate_id(int xmin,int xmax)
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

void WithLidar::measure_danda()
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

void WithLidar::display_distances(int person_num)
{
    //display distance and angle
    for(int i=0; i < person_num; i++)
    {
        std::stringstream ss;
        // ss << "person_" << i << " d: " << std::fixed << std::setprecision(2) << distance_[i] << " [m] / a: " << angle_[i]*180.0/M_PI - 180 -45;
        ss << "person_" << i << " d: " << std::fixed << std::setprecision(2) << distance_[i] << " [m] / a: " << angle_[i]*180/M_PI;
        std::string str = ss.str();
        cv::putText(input_image_, str, cv::Point(10, 30*(i+1)), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,255), 2, cv::LINE_AA);
        //put line on scan id
        double id_x = (id_[i]-b_)/a_;
        // std::cout << "id_x : " << id_x << std::endl;
        cv::line(input_image_, cv::Point(id_x,0), cv::Point(id_x,image_msg_.height), cv::Scalar(0,0,255), 2, cv::LINE_AA);
    }

    //put line on scan min and max
    cv::line(input_image_, cv::Point(image_width_/8,60), cv::Point(image_width_/8,image_msg_.height), cv::Scalar(255,0,0), 2, cv::LINE_AA);
    cv::line(input_image_, cv::Point(image_width_*7/8,0), cv::Point(image_width_*7/8,image_msg_.height), cv::Scalar(255,0,0), 2, cv::LINE_AA);

    //put line on brushee front
    cv::line(input_image_, cv::Point(image_width_*5/8,0), cv::Point(image_width_*5/8,image_msg_.height), cv::Scalar(0,255,0), 2, cv::LINE_AA);

    //publish image
    cv_bridge::CvImage cv_image;
    cv_image.header = image_msg_.header;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.image = input_image_;
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    pub_image_.publish(ros_image);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "with_lidar");
    WithLidar with_lidar;
    ros::spin();
    return 0;
}
