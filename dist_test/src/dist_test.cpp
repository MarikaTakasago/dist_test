// measure distance from depth image(midas image)
#include <dist_test/dist_test.h>

DistTest::DistTest() : private_nh_("~"),nh_()
{
    private_nh_.param("target_x", target_x, 678);
    private_nh_.param("target_y", target_y, 340);
    private_nh_.param("ref1_x", ref1_x_, 640);
    private_nh_.param("ref1_y", ref1_y_, 420);
    private_nh_.param("ref1_dist", ref1_dist_, 0.14); //LiDAR
    private_nh_.param("ref2_x", ref2_x_, 985);
    private_nh_.param("ref2_y", ref2_y_, 580);
    private_nh_.param("ref2_dist", ref2_dist_, 0.10); //pocket
    private_nh_.param("debug", debug_, true);

    sub_image_ = nh_.subscribe("/midas_topic", 1, &DistTest::image_callback, this);
    //for debug
    sub_theta_ = nh_.subscribe("/equirectangular/image_raw", 1, &DistTest::theta_callback, this);
    pub_image_ = nh_.advertise<sensor_msgs::Image>("/debug_image", 1);

}

//for debug
void DistTest::theta_callback(const sensor_msgs::ImageConstPtr& msg)
{
    if(debug_)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image = cv_ptr->image;
        cv::circle(image, cv::Point(target_x, target_y), 6, cv::Scalar(255,0,0), -1);
        cv::circle(image, cv::Point(ref1_x_, ref1_y_), 6, cv::Scalar(0,255,0), -1);
        cv::circle(image, cv::Point(ref2_x_, ref2_y_), 6, cv::Scalar(0,255,0), -1);
        display_distance(image,target_dist);
        //to rosmsg
        sensor_msgs::ImagePtr msg_image = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image).toImageMsg();
        pub_image_.publish(msg_image);
    }
}

void DistTest::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat input_image = cv_ptr->image;

    //normalize(0~255) image
    //search max value and min value
    double max_value = 0.0;
    double min_value = 0.0;
    cv::minMaxIdx(input_image, &min_value, &max_value);
    // std::cout<<"max_value: "<<max_value<<std::endl;
    // std::cout<<"min_value: "<<min_value<<std::endl;
    //normalize
    for(int i=0; i<input_image.rows; i++)
    {
        for(int j=0; j<input_image.cols; j++)
        {
            input_image.at<float>(i,j) = (input_image.at<float>(i,j) - min_value) / (max_value - min_value) * 255;
            input_image.at<float>(i,j) = (int)input_image.at<float>(i,j);
            //check if image is 0 ~255
            // if(input_image.at<float>(i,j)<0 || input_image.at<float>(i,j)>255)
            // {
            //     ROS_ERROR("\"%lf\"image is not 0~255", input_image.at<float>(i,j));
            // }
        }
    }

    input_image.convertTo(input_image, CV_8UC1);
    ROS_INFO("image received");
    measure_distance(input_image,target_x,target_y);
}

void DistTest::measure_distance(cv::Mat& img,int tx,int ty)
{
    // //set points
    // cv::Point2f ref1(ref1_x_, ref1_y_);
    // cv::Point2f ref2(ref2_x_, ref2_y_);
    // cv::Point2f target(tx, ty);
    //
    // //get distance in image
    // double ref1_imdist = 1.0/img.at<uchar>(ref1);
    // double ref2_imdist = 1.0/img.at<uchar>(ref2);
    // double target_imdist = 1.0/img.at<uchar>(target);

    //get distance in image (average of 3x3)
    double ref1_imdist = 0.0;
    double ref2_imdist = 0.0;
    double target_imdist = 0.0;
    for(int i=-1; i<2; i++)
    {
        for(int j=-1; j<2; j++)
        {
            ref1_imdist += img.at<uchar>(ref1_y_+i, ref1_x_+j);
            ref2_imdist += img.at<uchar>(ref2_y_+i, ref2_x_+j);
            target_imdist += img.at<uchar>(ty+i, tx+j);
        }
    }
    ref1_imdist /= 9.0;
    ref2_imdist /= 9.0;
    target_imdist /= 9.0;
    ref1_imdist = 1.0/ref1_imdist;
    ref2_imdist = 1.0/ref2_imdist;
    target_imdist = 1.0/target_imdist;

    //get distance in real world
    //calc y = ax + b
    double a = 0.0;
    double b = 0.0;
    linear_approximation(ref1_imdist,ref1_dist_,ref2_imdist,ref2_dist_,a,b);
    target_dist = a*(target_imdist) + b;

    if(!debug_) ROS_INFO("target_dist: %lf",target_dist);

    //for debug
    if(debug_)
    {
        ROS_INFO("y = %.2lf*x + %.2lf",a,b);
        ROS_INFO("ref1[%d][%d]_dist  : %.2lf -> %.2lf",ref1_x_,ref1_y_,1.0/ref1_imdist,ref1_dist_);
        ROS_INFO("ref2[%d][%d]_dist  : %.2lf -> %.2lf",ref2_x_,ref2_y_,1.0/ref2_imdist,ref2_dist_);
        ROS_INFO("target[%d][%d]_dist: %.2lf -> %.2lf",tx,ty,1.0/target_imdist,target_dist);
        ROS_INFO("----------------------------------------------------");
    }

}

void DistTest::linear_approximation(double x1,double y1,double x2,double y2,double& a,double& b)
{
    a = (y2-y1)/(x2-x1);
    b = y1 - a*x1;
}

void DistTest::display_distance(cv::Mat img, double dist)
{
    std::stringstream ss;
    ss << "distance: " << std::fixed << std::setprecision(2) << dist << " [m]";
    std::string str = ss.str();
    cv::putText(img, str, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,0), 2, cv::LINE_AA);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dist_test");
    DistTest dist_test;
    ros::spin();
    return 0;
}

