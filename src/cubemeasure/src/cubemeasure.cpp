#include "cubemeasure.hpp"

#define testimg_path "/home/lpga/finalhw_ros/src/cubemeasure/img/cube-1.jpg"

Mat raw_img = Mat::zeros(480, 640, CV_16UC1);
Mat raw_depth = Mat::zeros(480, 640, CV_16UC1);

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        raw_img = cv_ptr->image;
        // imshow("raw_img", raw_img);
        // waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'TYPE_16UC1'.", msg->encoding.c_str());
    }
}

void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        raw_depth = cv_ptr->image;
        // imshow("raw_depth", raw_depth);
        // waitKey(1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'TYPE_16UC1'.", msg->encoding.c_str());
    }
}

float cubemeasure(Mat rawimg, Mat depimg)
{
    rawimg = imread(testimg_path);
    // rawimg.resize(640, 480);
    // imshow("rawimg", rawimg);
    Mat grayimg;
    cvtColor(rawimg, grayimg, COLOR_BGR2GRAY);
    // imshow("grayimg", grayimg);
    // waitKey(1);

    // Mat blurimg;
    // blur(grayimg, blurimg, Size(10, 10));
    // imshow("blurimg", blurimg);
    // waitKey(1);

    namedWindow("thresh_tarimg", WINDOW_NORMAL);
    createTrackbar("thresh_tarimg_max", "thresh_tarimg", 0, 255);
    createTrackbar("thresh_tarimg_min", "thresh_tarimg", 0, 255);
    while (1)
    {
        Mat threshimg_max;
        Mat threshimg_min;
        Mat threshimg;
        int thresh_tarimg_value_max = getTrackbarPos("thresh_tarimg_max", "thresh_tarimg");
        int thresh_tarimg_value_min = getTrackbarPos("thresh_tarimg_min", "thresh_tarimg");

        threshold(grayimg, threshimg_max, 37, 255, THRESH_BINARY_INV);
        threshold(grayimg, threshimg_min, 28, 255, THRESH_BINARY);
        bitwise_and(threshimg_max, threshimg_min, threshimg);
        imshow("thresh_tarimg", threshimg);
        waitKey(1);
        blur(threshimg, threshimg, Size(50, 50));
        // waitKey(1);
        // imshow("thresh_tarimg", threshimg);
        threshold(threshimg, threshimg, 127, 255, THRESH_BINARY);
        imshow("thresh_tarimg", threshimg);
        
        // imshow("threshimg_max", threshimg_max);
        // imshow("threshimg_min", threshimg_min);
        waitKey(1);
    } // 测量阈值

    // imshow("threshimg", threshimg);
    // waitKey(1);
    return 0;
}

int main(int argc, char **argv)
{

    float length = cubemeasure(raw_img, raw_depth);
    return 0;
}

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "cubemeasure");
//     ros::NodeHandle nh;
//     ros::Subscriber sub = nh.subscribe("/camera/color/image_raw", 1, imageCallback);
//     ros::Subscriber sub = nh.subscribe("/camera/depth/image_raw", 1, depthCallback);
//     ros::Publisher pub = nh.advertise<std_msgs::String>("cubemeasure_result_str", 1);
//     ros::Rate loop_rate(10);

//     return 0;
// }