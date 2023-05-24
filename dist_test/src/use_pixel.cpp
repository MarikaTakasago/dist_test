#include <dist_test/use_pixel.h>

WithLidar::WithLidar() : private_nh_("~")
{
    private_nh_.param("scan_line_sum", scan_line_sum_, 2161);
    private_nh_.param("scan_angle", scan_angle_, 270);
    private_nh_.param("laser_num", laser_num_, 10);
    private_nh_.param("display", display_, false);
    private_nh_.param("conf_th_", conf_th_, 0.5);
    private_nh_.param("upper", upper_, 0.3);
    private_nh_.param("lower", lower_, 0.1);

    sub_scan_ = nh_.subscribe("/scan",1,&WithLidar::scan_callback,this);
    sub_image_ = nh_.subscribe("/detected_image",1,&WithLidar::image_callback,this);
    sub_bbox_ = nh_.subscribe("/masks",1,&WithLidar::bbox_callback,this);

    pub_image_ = nh_.advertise<sensor_msgs::Image>("/with_lidar",1);
    pub_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/person_poses",1);

    pic_id_min_ = pic_id_max_ = box_id_min_ = box_id_max_ = 0;
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
    bool get_person = false;
    if(masks_.masks.size() == 0)
    {
        std::cout<<"no person"<<std::endl;
        if(get_image_ && display_) display_distances(0);
    }
    else get_person = true;

    if(get_image_ && get_scan_ && get_person)
    {
        get_bbox_ = true;

        //reset vector and counter
        int person_num = 0;
        distance_.clear();
        angle_.clear();
        person_poses_.poses.clear();
        person_poses_.header.stamp = scan_.header.stamp;

        for(auto mask : masks_.masks)
        {
            //get pixel
            mask_image_msg = mask.mask;
            //to cv image
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(mask_image_msg,sensor_msgs::image_encodings::MONO8);
            }
            catch(cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            mask_image_ = cv_ptr->image;
            get_masked_pixels(mask_image_);
            int masked_size = masked_pixels_.size();

            //get bbox
            camera_apps_msgs::BoundingBox bbox = mask.bounding_box;

            //if label is not person skip
            if(bbox.label != "person" || bbox.confidence < conf_th_) continue;

            //if person is not in the scan angle : skip
            int xmin = bbox.xmin+masked_pixels_[0];
            int xmax = bbox.xmin+masked_pixels_[masked_size-1];
            if(xmax < image_width_/8 || image_width_*7/8 < xmin)
            {
                calculate_id_pic(bbox.xmin,masked_size);
                std::cout<<"not in scan angle : " << angle_[person_num]*180/M_PI << "deg" <<std::endl;
                distance_.push_back(-1.0);

                tf::Quaternion q;
                q.setRPY(0,0,angle_[person_num]);
                geometry_msgs::Quaternion quat_msg;
                tf::quaternionTFToMsg(q,quat_msg);

                geometry_msgs::Pose pose;
                pose.position.x = 0;
                pose.position.y = 0;
                pose.position.z = 0;
                pose.orientation = quat_msg;
                person_poses_.poses.push_back(pose);

                person_num++;
                continue;
            }
            if(xmin < image_width_/8) xmin = image_width_/8;
            if(image_width_*7/8 < xmax) xmax = image_width_*7/8;

            calculate_id_pic(bbox.xmin,masked_size);
            calculate_id_box(bbox.xmin,bbox.xmax);
            measure_danda(); //calculate distance and angle

            //for debug
            std::cout << "x range : " << xmin << "," << xmax<< std::endl;
            std::cout << "scan_id : " << (pic_id_min_ + pic_id_max_)/2.0 << std::endl;
            std::cout << "distance : " << distance_[person_num] << std::endl;
            std::cout << "angle : " << angle_[person_num]*180/M_PI << std::endl;
            std::cout<<"==========================================="<<std::endl;

            //angle to quaternion
            tf::Quaternion q;
            q.setRPY(0,0,angle_[person_num]);
            geometry_msgs::Quaternion quat_msg;
            tf::quaternionTFToMsg(q,quat_msg);

            //to msg
            geometry_msgs::Pose pose;
            pose.position.x = distance_[person_num]*cos(angle_[person_num]) - 0.07;
            pose.position.y = distance_[person_num]*sin(angle_[person_num]) - 0.07;
            pose.position.z = 0;
            pose.orientation = quat_msg;
            person_poses_.poses.push_back(pose);

            person_num++;
        }
        if(display_) display_distances(person_num);
        //pub poses
        pub_poses_.publish(person_poses_);

    }
}

void WithLidar::calculate_id_pic(int bbox_min,int size)
{
    //calc id from grid x
    // make id list
    pic_id_.clear();
    double lidar_fi_min = 0.0;
    double lidar_fi_max = 0.0;
    std::cout << "id : ";
    for(int i=size-1; i>=0; i--)
    {
        int x = bbox_min + masked_pixels_[i];
        double id_lidar_fi = M_PI * x * 2.0/image_width_;
        double id_angle = id_lidar_fi - M_PI - M_PI/4.0;
        int id = -8 * id_angle * 180/M_PI + 720;
        if(i == 0)
        {
            pic_id_min_ = id;
            lidar_fi_min = id_lidar_fi;
        }
        if(i == size-1)
        {
            pic_id_max_ = id;
            lidar_fi_max = id_lidar_fi;
        }
        if(id < 0 || scan_line_sum_ < id) continue;
        pic_id_.push_back(id);
    }
    std::cout << "----pic----" << std::endl;
    std::cout << "id_min : " << pic_id_min_ << std::endl;
    std::cout << "id_max : " << pic_id_max_ << std::endl;
    std::cout << "fi_min : " << (lidar_fi_min) * 180/M_PI << std::endl;
    std::cout << "fi_max : " << (lidar_fi_max) * 180/M_PI << std::endl;
    double lidar_fi_ave = (lidar_fi_min + lidar_fi_max)/2.0;
    // double fi_ave = lidar_fi_ave + M_PI/4.0;
    // if(fi_ave > 2*M_PI) fi_ave -= 2*M_PI;
    if(lidar_fi_ave > 2*M_PI) lidar_fi_ave -= 2*M_PI;
    if(lidar_fi_min > 2*M_PI) lidar_fi_min -= 2*M_PI;
    if(lidar_fi_max > 2*M_PI) lidar_fi_max -= 2*M_PI;
    //0~360 -> -180 ~ 180
    // if(fi_ave > M_PI) fi_ave -= 2*M_PI;
    // if(lidar_fi_ave > M_PI) lidar_fi_ave -= 2*M_PI;
    double angle = lidar_fi_ave - M_PI - M_PI/4.0;
    double angle_min = lidar_fi_min - M_PI - M_PI/4.0;
    double angle_max = lidar_fi_max - M_PI - M_PI/4.0;
    if(angle < -M_PI) angle += 2*M_PI;
    angle_.push_back(-1.0*angle);

}

void WithLidar::calculate_id_box(int xmin,int xmax)
{
    std::cout<<"----bbox----"<<std::endl;
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
    // angle_.push_back(angle);
    box_id_max_ = -8*angle_min*180/M_PI + 720;
    box_id_min_ = -8*angle_max*180/M_PI + 720;
    std::cout << "id_min : " << box_id_min_ << std::endl;
    std::cout << "id_max : " << box_id_max_ << std::endl;

}

void WithLidar::measure_danda()
{
    //danda means distance and angle
    //calc distance from id
    //use masked pixels
    std::vector<double> scan_data;
    scan_data.clear();
    int id_size = pic_id_.size();
    for(int i=0;i<=id_size;i++)
    {
        int id = pic_id_[i];
        if(id < 0 || scan_line_sum_ < id) continue;
        if(scan_.ranges[id] > scan_.range_min && scan_.ranges[id] < scan_.range_max)
        {
            scan_data.push_back(scan_.ranges[id]);
        }
    }
    std::cout << std::endl;
    if(scan_data.size() == 0) return;
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
    int lim = laser_num_;
    if(scan_data.size() < lim) lim = scan_data.size();
    double sum = 0;
    for(int i=0;i<lim;i++)
    {
        sum += scan_data[i];
    }
    double pic_d = sum/lim;

    //use bbox
    std::vector<double> scan_data_b;
    scan_data_b.clear();
    for(int i=box_id_min_;i<=box_id_max_;i++)
    {
        if(scan_.ranges[i] > scan_.range_min && scan_.ranges[i] < scan_.range_max)
        {
            scan_data_b.push_back(scan_.ranges[i]);
        }
    }
    if(scan_data_b.size() == 0) return;
    //sort min to max
    for(int i=0;i<scan_data_b.size()-1;i++)
    {
        for(int j=i+1;j<scan_data_b.size();j++)
        {
            if(scan_data_b[i] > scan_data_b[j])
            {
                double tmp = scan_data_b[i];
                scan_data_b[i] = scan_data_b[j];
                scan_data_b[j] = tmp;
            }
        }
    }
    // std::sort(scan_data.begin(),scan_data.end());
    //calc average of minimum 10 points
    double sum_b = 0;
    int count_b = 0;
    int lim_b = laser_num_;
    if(scan_data_b.size() < laser_num_) lim_b = scan_data_b.size();
    for(int i = 0; i < lim_b; i++)
    {
        sum_b += scan_data_b[i];
        count_b++;
    }
    double bbox_d = sum_b / count_b;

    double dist = std::min(pic_d,bbox_d);

    distance_.push_back(dist);
    //calc distance
    //median
    // double distance = scan_data[scan_data.size()/2];
    //average of 25% ~ 75%
    // double distance_sum = 0;
    // int distance_num = 0;
    // for(int i=scan_data.size()/4;i<scan_data.size()*3/4;i++)
    // {
    //     distance_sum += scan_data[i];
    //     distance_num++;
    // }
    // double distance = distance_sum/distance_num;
    // std::cout<<"distance : "<<distance<<std::endl;
    // distance_.push_back(distance);
}

void WithLidar::get_masked_pixels(cv::Mat img)
{
    int height = img.rows;
    int min_h = (1.0-lower_) * height;
    int max_h = (1.0-upper_) * height;
    std::cout << "min_h : " << min_h << std::endl;
    std::cout << "max_h : " << max_h << std::endl;
    //get white pixels
    bool skip = false;
    masked_pixels_.clear();
    for(int i = max_h; i < min_h; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            for(int k = 0;k<masked_pixels_.size();k++)
            {
                if(masked_pixels_[k] == j)
                {
                    skip = true;
                    break;
                }
            }
            if(img.at<uchar>(i,j) == 255 && !skip)
            {
                masked_pixels_.push_back(j);
            }
            skip = false;
        }
    }
    //sort min to max
    for(int i=0;i<masked_pixels_.size()-1;i++)
    {
        for(int j=i+1;j<masked_pixels_.size();j++)
        {
            if(masked_pixels_[i] > masked_pixels_[j])
            {
                int tmp = masked_pixels_[i];
                masked_pixels_[i] = masked_pixels_[j];
                masked_pixels_[j] = tmp;
            }
        }
    }
}

void WithLidar::display_distances(int person_num)
{
    //draw rectangle laser range alpha = 10
    int start_x = image_width_/8;
    int start_y = 0;
    int end_x = image_width_/8*7;
    int end_y = 640;
    cv::Mat roi = input_image_(cv::Rect(0,0,start_x,end_y));
    cv::Mat color(roi.size(),CV_8UC3,cv::Scalar(0,0,255));
    double alpha = 0.8;
    cv::addWeighted(roi,alpha,color,1.0-alpha,0.0,roi);
    cv::Mat roi2 = input_image_(cv::Rect(end_x,0,image_width_-end_x,end_y));
    cv::Mat color2(roi2.size(),CV_8UC3,cv::Scalar(0,0,255));
    cv::addWeighted(roi2,alpha,color2,1.0-alpha,0.0,roi2);

    //display distance and angle
    if (person_num > 0)
    {
        for(int i=0; i < person_num; i++)
        {
            std::stringstream ss;
            if(distance_[i]>=0)
            {
                ss << "Person" << i << ": " << std::fixed << std::setprecision(2) << distance_[i] << " m , " << angle_[i]*180/M_PI << " deg";
            }
            else
            {
                ss << "Person" << i << ": OutOfRange, " << std::fixed << std::setprecision(2) << angle_[i]*180/M_PI << " deg";
            }

            std::string str = ss.str();
            cv::putText(input_image_, str, cv::Point(10, 30*(i+1)), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,0,0), 2, cv::LINE_AA);
        }
    }

    //put line on brushee front and put text "front"
    cv::putText(input_image_, "front", cv::Point(image_width_*5/8-35,30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,0), 2, cv::LINE_AA);
    cv::line(input_image_, cv::Point(image_width_*5/8,50), cv::Point(image_width_*5/8,500), cv::Scalar(0,255,0), 2, cv::LINE_AA);

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
    ros::init(argc, argv, "use_pixel");
    WithLidar use_pixel;
    ros::spin();
    return 0;
}
