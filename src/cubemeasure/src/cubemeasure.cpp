#include "cubemeasure.hpp"

#define testimg_path "/home/lpga/finalhw_ros/src/cubemeasure/img/cube-1.jpg"

Mat raw_img;
Mat raw_depth;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        raw_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        imshow("raw_img", raw_img);
        // waitKey(0);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        raw_depth = cv_bridge::toCvCopy(msg, "mono8")->image;
        imshow("raw_depth", raw_depth);
        // waitKey(0);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
    }
}

void predispose(InputArray rawimg, OutputArray outimg, int border_value, int thresh_max, int thresh_min)
{
    Mat grayimg;
    cvtColor(rawimg, grayimg, COLOR_BGR2GRAY);
    // imshow("grayimg", grayimg);
    Mat threshimg_max;
    Mat threshimg_min;
    Mat threshimg;
    threshold(grayimg, threshimg_max, thresh_max, 255, THRESH_BINARY_INV);
    threshold(grayimg, threshimg_min, thresh_min, 255, THRESH_BINARY);
    bitwise_and(threshimg_max, threshimg_min, threshimg);
    erode(threshimg, threshimg, Mat(), Point(-1, -1), border_value);
    imshow("threshimg", threshimg);
    dilate(threshimg, outimg, Mat(), Point(-1, -1), border_value);

    // blur(threshimg, threshimg, Size(blur_size, blur_size));
    // threshold(threshimg, outimg, 127, 255, THRESH_BINARY);

    imshow("outimg", outimg);
}

vector<Point> contoursfilter(vector<vector<Point>> contours)
{

    vector<Point> simplifiedcontours;
    for (int i = 0; i < contours.size(); i++)
    {
        if (i == 0)
        {
            simplifiedcontours = contours[i];
        }
        else
        {
            if (contours[i].size() > simplifiedcontours.size())
            {
                simplifiedcontours = contours[i];
            }
        }
    }
    approxPolyDP(simplifiedcontours, simplifiedcontours, 5, true);
    vector<int> hull;
    convexHull(simplifiedcontours, hull);
    vector<Point> hull_points;
    for (int i = 0; i < hull.size(); i++)
    {
        hull_points.push_back(simplifiedcontours[hull[i]]);
    }
    cv::Point center_point = cv::Point(0, 0);
    for (int i = 0; i < hull_points.size(); i++)
    {
        center_point += hull_points[i];
    }
    center_point *= (1. / hull_points.size());
    simplifiedcontours = hull_points;

    // int minidelta_xy = 30;

    // for (int i = 0; i < simplifiedcontours.size(); i++)
    // {
    //     if (i == 0)
    //     {
    //         if ((abs(simplifiedcontours[i].x - simplifiedcontours[simplifiedcontours.size() - 1].x) < minidelta_xy) && (abs(simplifiedcontours[i].y - simplifiedcontours[simplifiedcontours.size() - 1].y) < minidelta_xy))
    //         {
    //             if (simplifiedcontours[i].y < center_point.y)
    //             {
    //                 if (simplifiedcontours[i].y < simplifiedcontours[i - 1].y)
    //                 {
    //                     simplifiedcontours[i - 1].x = simplifiedcontours[i].x;
    //                     simplifiedcontours[i - 1].y = simplifiedcontours[i].y;
    //                 }
    //             }
    //             else
    //             {
    //                 if (simplifiedcontours[i].y > simplifiedcontours[i - 1].y)
    //                 {
    //                     simplifiedcontours[i - 1].x = simplifiedcontours[i].x;
    //                     simplifiedcontours[i - 1].y = simplifiedcontours[i].y;
    //                 }
    //             }
    //             simplifiedcontours.erase(simplifiedcontours.begin() + i);
    //         }
    //     }
    //     else
    //     {
    //         if ((abs(simplifiedcontours[i].x - simplifiedcontours[i - 1].x) < minidelta_xy) && (abs(simplifiedcontours[i].y - simplifiedcontours[i - 1].y) < minidelta_xy))
    //         {
    //             if (simplifiedcontours[i].y < center_point.y)
    //             {
    //                 if (simplifiedcontours[i].y < simplifiedcontours[i - 1].y)
    //                 {
    //                     simplifiedcontours[i - 1].x = simplifiedcontours[i].x;
    //                     simplifiedcontours[i - 1].y = simplifiedcontours[i].y;
    //                 }
    //             }
    //             else
    //             {
    //                 if (simplifiedcontours[i].y > simplifiedcontours[i - 1].y)
    //                 {
    //                     simplifiedcontours[i - 1].x = simplifiedcontours[i].x;
    //                     simplifiedcontours[i - 1].y = simplifiedcontours[i].y;
    //                 }
    //             }
    //             simplifiedcontours.erase(simplifiedcontours.begin() + i);
    //         }
    //     }
    //     cout << "simplifiedcontours[" << i << "]: " << simplifiedcontours[i] << endl;
    // }
    return simplifiedcontours;
}

float p2l_distance(Point2f point1, Point2f point2, Point2f point)
{
    Point2f vec1 = point1 - point2;
    Point2f vec2 = point - point2;
    float cross = vec1.x * vec2.y - vec1.y * vec2.x;
    float length = cv::norm(vec1);
    float distance = cross / length;
    return distance;
}

float cubemeasure(Mat rawimg, Mat depimg)
{
    vector<KeyPoint> keypoints;
    rawimg = imread(testimg_path);
    // rawimg.resize(640, 480);
    // imshow("rawimg", rawimg);
    // waitKey(1);
    Mat threshimg;
    predispose(rawimg, threshimg, 5, 36, 28);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(threshimg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    vector<Point> target_contours;
    target_contours = contoursfilter(contours);
    for (int i = 0; i < target_contours.size(); i++)
    {
        keypoints.push_back(KeyPoint(target_contours[i], 1));
        drawKeypoints(rawimg, keypoints, rawimg, Scalar(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        putText(rawimg, to_string(target_contours[i].x) + "," + to_string(target_contours[i].y) + "num: " + to_string(i), target_contours[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
    }

    Rect rect = boundingRect(target_contours);
    int originx = rect.x;
    int originy = rect.y;
    int originwidth = rect.width;
    int originheight = rect.height;
    int shrinkwidth = originwidth * 0.2;
    int shrinkheight = originheight * 1.2;
    Rect shrinkrect(rect.x + (originwidth - shrinkwidth) / 2, rect.y + (originheight - shrinkheight) / 2, shrinkwidth, shrinkheight);

    Mat Mask = Mat::zeros(rawimg.size(), CV_8UC1);
    rectangle(Mask, shrinkrect, Scalar(255), -1);
    bitwise_and(threshimg, Mask, threshimg);

    findContours(threshimg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    // cout << "contours.size(): " << contours.size() << endl;
    target_contours = contoursfilter(contours);
    vector<Point> upcontours;
    vector<Point> downcontours;
    for (int i = 0; i < target_contours.size(); i++)
    {
        if (target_contours[i].y < shrinkrect.y + shrinkrect.height / 2 && upcontours.size() < 2)
        {
            upcontours.push_back(target_contours[i]);
        }
        else if (target_contours[i].y > shrinkrect.y + shrinkrect.height / 2 && downcontours.size() < 2)
        {
            downcontours.push_back(target_contours[i]);
        }
    }
    line(rawimg, upcontours[0], upcontours[upcontours.size() - 1], Scalar(255, 255, 255), 1);
    line(rawimg, downcontours[0], downcontours[downcontours.size() - 1], Scalar(255, 255, 255), 1);

    float dist = (p2l_distance(upcontours[0], upcontours[upcontours.size() - 1], downcontours[0]) + p2l_distance(upcontours[0], upcontours[upcontours.size() - 1], downcontours[downcontours.size() - 1])) / 2;
    cout << "dist: " << fabsf(dist) << endl;

    // rectangle(rawimg, srinkrect, Scalar(0, 255, 0), 2);
    // imshow("rawimg", rawimg);
    // waitKey(1);

    // for (int i = 0; i < target_contours.size(); i++)
    // {
    //     keypoints.push_back(KeyPoint(target_contours[i], 1));
    //     drawKeypoints(rawimg, keypoints, rawimg, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    //     putText(rawimg, to_string(target_contours[i].x) + "," + to_string(target_contours[i].y) + "num: " + to_string(i), target_contours[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
    // }
    // drawContours(rawimg, vector<vector<Point>>{target_contours}, -1, Scalar(255, 255, 255), 1);

    //

    // float theta[target_contours.size()] = {0};
    // for (int i = 0; i < target_contours.size(); i++)
    // {
    //     if (i == 0)
    //     {
    //         theta[0] = atan2(target_contours[i].y - target_contours[target_contours.size() - 1].y, target_contours[i].x - target_contours[target_contours.size() - 1].x);
    //     }
    //     else
    //     {
    //         theta[i] = atan2(target_contours[i].y - target_contours[i - 1].y, target_contours[i].x - target_contours[i - 1].x);
    //     }
    //     cout << "theta[" << i << "]: " << theta[i] << endl;
    // }
    // float theta_dv = 5.f;

    // for (int i = 0; i < target_contours.size(); i++)
    // {
    //     keypoints.push_back(KeyPoint(target_contours[i], 1));
    //     drawKeypoints(rawimg, keypoints, rawimg, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    //     putText(rawimg, to_string(target_contours[i].x) + "," + to_string(target_contours[i].y) + "num: " + to_string(i), target_contours[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
    // }
    // drawContours(rawimg, vector<vector<Point>>{target_contours}, -1, Scalar(255, 255, 255), 1);

    imshow("rawimg", rawimg);
    waitKey(0);

    return 0;
}

// int main(int argc, char **argv)
// {

//     float length = cubemeasure(raw_img, raw_depth);
//     waitKey(0);
//     return 0;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cubemeasure");
    ros::NodeHandle nh;
    ros::Subscriber sub_rgb = nh.subscribe("/camera2/rgb/image_raw", 1, imageCallback);
    ros::Subscriber sub_depth = nh.subscribe("/camera2/depth/image_raw", 1, depthCallback);
    ros::Publisher pub = nh.advertise<std_msgs::String>("cubemeasure_result_str", 1);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        // float length = cubemeasure(raw_img, raw_depth);
        // imshow("raw_img", raw_img);
        // imshow("raw_depth", raw_depth);
        std_msgs::String msg;
        std::stringstream ss;
        // ss << length;
        msg.data = ss.str();
        pub.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}