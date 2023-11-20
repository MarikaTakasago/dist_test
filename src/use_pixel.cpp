#include <dist_test/use_pixel.h>

WithLidar::WithLidar() : private_nh_("~")
{
    private_nh_.param<int>("referenced_laser_num", param_.referenced_laser_num, 50);
    private_nh_.param<float>("offset_angle_camera_to_lidar", param_.offset_angle_camera_to_lidar, 0);
    private_nh_.param<std::string>("masks_topic_name", param_.masks_topic_name,"/mask_rcnn/masks");
    private_nh_.param<std::string>("scan_topic_name", param_.scan_topic_name,"/scan");


    scan_sub_ptr_ = std::make_unique<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, param_.scan_topic_name, 30);
    masks_sub_ptr_ = std::make_unique<message_filters::Subscriber<camera_apps_msgs::Masks>>(nh_, param_.masks_topic_name, 5);
    // MySyncPolicyの引数はおそらくためとくmessageの数
    synchronizer_ptr_ = std::make_unique<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(30), *scan_sub_ptr_, *masks_sub_ptr_);  
    synchronizer_ptr_->registerCallback(&WithLidar::synchro_callback, this);

    person_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/use_pixel/person_poses",1);
}
void WithLidar::synchro_callback(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                 const camera_apps_msgs::MasksConstPtr& masks_msg)
{
    geometry_msgs::PoseArray person_poses;
    person_poses.header = scan_msg->header;

    for(const auto& mask_and_bounding_box: masks_msg->masks)
    {
        const auto mask_cv_img = convert_img_msg_to_cv(mask_and_bounding_box.mask);
        if(!mask_cv_img.has_value()) continue;

        //この時点ではidxはmask画像におけるidx
        std::vector<int> most_masked_col_idx = calc_most_masked_col_idxs(
                mask_cv_img.value(), param_.referenced_laser_num);
        //idxをカメラ画像におけるidxに変換
        for(auto& col_id: most_masked_col_idx)
            col_id += mask_and_bounding_box.bounding_box.xmin;

        // この時点ではanglesは全方位カメラからみたangle
        std::vector<float> angles = col_idxs_to_angles(most_masked_col_idx, masks_msg->width);
        // anglesをlidarにからみたanglesに変換
        for(auto& angle: angles)
            angle = convert_angle_camera_to_lidar(angle, param_.offset_angle_camera_to_lidar);

        const auto pose = calc_pose_from_angles(*scan_msg, angles);
        if(!pose.has_value()) continue;
        person_poses.poses.push_back(pose.value());
    }

    person_poses_pub_.publish(person_poses);
}

// maskされたpixelが多いindexを，多い順にreferenced_laser_num分返す
// indexはmask画像におけるindexであり，カメラ画像におけるindexではないことに注意
std::vector<int> WithLidar::calc_most_masked_col_idxs(const cv::Mat& mask, int referenced_laser_num)
{
    cv::Mat masked_pixel_counts_of_each_col;
    cv::reduce((mask== 255) / 255, masked_pixel_counts_of_each_col, 0, cv::REDUCE_SUM, CV_32S);
    
    // 第一要素がマスクされたピクセルの数，第二要素が列番号
    std::vector<std::pair<int, int>> pairs_of_count_and_idx; // pairs of masked pixel count and col idx
    pairs_of_count_and_idx.reserve(masked_pixel_counts_of_each_col.cols);

    masked_pixel_counts_of_each_col.forEach<int>([&](int& count, const int position[2]) -> void{
        pairs_of_count_and_idx.push_back({count, position[1]});
    });

    // vector<pair>のソートは第一要素-->第二要素の順でソートされる
    // マスクされたピクセル数に関して降順でsortする
    std::sort(pairs_of_count_and_idx.begin(), pairs_of_count_and_idx.end(),
            std::greater<std::pair<int, int>>{});

    std::vector<int> most_masked_col_idx;
    most_masked_col_idx.reserve(referenced_laser_num);
    for(int i=0, end=std::min((int)pairs_of_count_and_idx.size(), referenced_laser_num); i<end; i++)
        most_masked_col_idx.push_back(pairs_of_count_and_idx[i].second);

    return most_masked_col_idx;
}

// 全方位画像における列番号からカメラからの方位[rad]を計算
std::vector<float> WithLidar::col_idxs_to_angles(const std::vector<int>& col_idxs, int img_width)
{
    std::vector<float> angles;
    angles.reserve(col_idxs.size());

    // col=0のときangle=pi, col=img_widthのときangle=-piになるように計算する
    for(const auto& col_idx: col_idxs)
        angles.push_back(M_PI - 2*M_PI*((float)col_idx/img_width));

    return angles;
}

// 全方位カメラから見た方位を，lidarから見た方位に変える
float WithLidar::convert_angle_camera_to_lidar(float angle_at_camera, float offset_angle_camera_to_lidar)
{
    float angle_at_lidar = adjust_angle(angle_at_camera - offset_angle_camera_to_lidar);

    return angle_at_lidar;
}

float WithLidar::adjust_angle(float angle)
{
    if(angle > M_PI) return angle - 2*M_PI;
    if(angle < -M_PI) return angle + 2*M_PI;

    return angle;
}

std::optional<geometry_msgs::Pose> WithLidar::calc_pose_from_angles(
        const sensor_msgs::LaserScan& scan, const std::vector<float>& angles)
{
    std::pair<float, float> sum_x_and_y = {0, 0};
    int valid_point_count = 0;
    for(const auto& angle: angles)
    {
        auto point = calc_point_from_angle(scan, angle);
        if(!point.has_value()) continue;

        auto [x, y] = point.value();
        sum_x_and_y.first += x;
        sum_x_and_y.second += y;

        valid_point_count++;
    }

    if(valid_point_count == 0) return std::nullopt;

    float ave_x = sum_x_and_y.first / valid_point_count;
    float ave_y = sum_x_and_y.second / valid_point_count;

    geometry_msgs::Pose pose;
    pose.position.x = ave_x;
    pose.position.y = ave_y;

    return pose;
}

// angleがlidarの範囲外もしくはセンサ値が無効値だったら無効値を返す
std::optional<std::pair<float, float>> WithLidar::calc_point_from_angle(
        const sensor_msgs::LaserScan& scan, float angle)
{
    if(!(angle >= scan.angle_min && angle <= scan.angle_max)) return std::nullopt;

    // angleがranges[]のどこに対応するか計算する  
    int idx_at_ranges = std::min((int)((angle - scan.angle_min) / scan.angle_increment), (int)scan.ranges.size() - 1);
    float range = scan.ranges[idx_at_ranges];
    if(!(range >= scan.range_min && range <= scan.range_max)) return std::nullopt;

    return std::make_pair(range * std::cos(angle), range * std::sin(angle));
}

std::optional<cv::Mat> WithLidar::convert_img_msg_to_cv(const sensor_msgs::Image& img_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return std::nullopt;
    }

    return cv_ptr->image;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "use_pixel");
    WithLidar use_pixel;
    ros::spin();
    return 0;
}
