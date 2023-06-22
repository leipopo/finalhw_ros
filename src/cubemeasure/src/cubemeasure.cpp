#include "cubemeasure.hpp"

// #define testimg_path "/home/lpga/finalhw_ros/src/cubemeasure/img/cube-1.jpg"

Mat raw_img;
Mat raw_depth;

Mat camerargbinfo;
Mat cameradepthinfo;

float cubelength = 1.0;

NF nf = {0, 0.5, 0.01};

ros::Subscriber sub_camerargbinfo;
ros::Subscriber sub_depth;
ros::Subscriber sub_rgb;

float numberfusion(float x_n_1, NF *nf)
{
    float K_n = nf->E_est_n_1 / (nf->E_est_n_1 + nf->E_mea_n);
    float x_n = nf->x_n_1 + K_n * (x_n_1 - nf->x_n_1);
    nf->E_est_n_1 = MAX(0.005, (1 - K_n) * nf->E_est_n_1);
    nf->x_n_1 = x_n;
    // nf->E_mea_n = 1-nf->E_est_n_1;
    cout << "k_n:" << K_n << endl;
    cout << "mea:" << nf->E_mea_n << endl;
    cout << "est:" << nf->E_est_n_1 << endl;
    return x_n;
}

float getcubelength(float d[2], float cublen, NF *nf)
{
    if (d[0] >= 0.98 && d[1] >= 0.98 && d[0] < 2.02 && d[1] < 2.02 && fabsf(d[0] - d[1]) < 0.1)
    {
        cublen = numberfusion(d[0], nf);
        cublen = numberfusion(d[1], nf);
        
        // cublen = numberfusion(MIN(d[0], d[1]), nf);
    }
    else if (d[1] >= 0.98 && d[1] < 2.02 && fabsf(d[1] - cublen) < 0.1)
    {
        // if (fabsf(d[1] - cublen) < 0.1)
        // {
        cublen = numberfusion(d[1], nf);
        // }
        // else
        // {
        //     cublen = numberfusion(cublen, nf);
        // }
    }
    else if (d[0] >= 0.98 && d[0] < 2.02 && fabsf(d[0] - cublen) < 0.1)
    {
        // if (fabsf(d[0] - cublen) < 0.1)
        // {
        cublen = numberfusion(d[0], nf);
        // }
        // else
        // {
        //     cublen = numberfusion(cublen, nf);
        // }
    }
    else
    {
        nf->E_est_n_1 = 0.5;
        nf->E_mea_n = 0.01;
        cout << "reset" << endl;
        cublen = numberfusion(cublen, nf);
    }
    return cublen;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        raw_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
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
        raw_depth = cv_bridge::toCvCopy(msg, "32FC1")->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
    }
}

void camerargbinfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    camerargbinfo = Mat(3, 3, CV_64FC1, (void *)msg->K.data()).clone();
    sub_camerargbinfo.shutdown();
}

Point3f pixel2camera(Point2f pixel, float depth, Mat camerainfo)
{
    float fx = camerainfo.at<double>(0, 0);
    float fy = camerainfo.at<double>(1, 1);
    float cx = camerainfo.at<double>(0, 2);
    float cy = camerainfo.at<double>(1, 2);
    Point3f camera_point;
    camera_point.x = (pixel.x - cx) * depth / fx;
    camera_point.y = (pixel.y - cy) * depth / fy;
    camera_point.z = depth;
    return camera_point;
}

void predispose(InputArray rawimg, OutputArray outimg, int border_value, int thresh_max, int thresh_min, int thresh_green_min, int thresh_blue_green_max)
{
    Mat imageWithout_Red_blue = rawimg.getMat().clone();

    for (int i = 0; i < imageWithout_Red_blue.rows; i++)
    {
        for (int j = 0; j < imageWithout_Red_blue.cols; j++)
        {
            if (imageWithout_Red_blue.at<cv::Vec3b>(i, j)[1] < thresh_green_min || imageWithout_Red_blue.at<cv::Vec3b>(i, j)[0] > thresh_blue_green_max || imageWithout_Red_blue.at<cv::Vec3b>(i, j)[2] > thresh_blue_green_max)
            {
                imageWithout_Red_blue.at<cv::Vec3b>(i, j)[1] = 0; // 绿色通道置零
            }
            imageWithout_Red_blue.at<cv::Vec3b>(i, j)[2] = 0; // 红色通道置零
            imageWithout_Red_blue.at<cv::Vec3b>(i, j)[0] = 0; // 蓝色通道置零
        }
    }

    Mat grayimg;
    cvtColor(imageWithout_Red_blue, grayimg, COLOR_BGR2GRAY);
    Mat threshimg_max;
    Mat threshimg_min;
    Mat threshimg;
    threshold(grayimg, threshimg_max, thresh_max, 255, THRESH_BINARY_INV);
    threshold(grayimg, threshimg_min, thresh_min, 255, THRESH_BINARY);
    bitwise_and(threshimg_max, threshimg_min, threshimg);
    erode(threshimg, threshimg, Mat(), Point(-1, -1), border_value);
    imshow("threshimg", threshimg);
    dilate(threshimg, outimg, Mat(), Point(-1, -1), border_value);

    // imshow("outimg", outimg);
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

void findmaxsizeContours(vector<vector<Point>> contours, vector<Point> &maxsizeContours)
{
    int maxsize = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        if (contours[i].size() > maxsize)
        {
            maxsize = contours[i].size();
            maxsizeContours = contours[i];
        }
    }
}

vector<Point> contoursfilter(vector<vector<Point>> contours)
{

    vector<Point> simplifiedcontours;
    findmaxsizeContours(contours, simplifiedcontours);
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

    std::sort(hull_points.begin(), hull_points.end(), [center_point](cv::Point a, cv::Point b)
              { return std::atan2(a.y - center_point.y, a.x - center_point.x) < std::atan2(b.y - center_point.y, b.x - center_point.x); });

    simplifiedcontours = hull_points;
    return simplifiedcontours;
}

Point findheighestPoint(vector<Point> contours)
{
    Point heighestPoint = contours[0];
    for (int i = 0; i < contours.size(); i++)
    {
        if (contours[i].y < heighestPoint.y)
        {
            heighestPoint = contours[i];
        }
    }
    return heighestPoint;
}

Point findlowestPoint(vector<Point> contours)
{
    Point lowestPoint = contours[0];
    for (int i = 0; i < contours.size(); i++)
    {
        if (contours[i].y > lowestPoint.y)
        {
            lowestPoint = contours[i];
        }
    }
    return lowestPoint;
}

float findaverheight(vector<Point> contours)
{
    float aver_height = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        aver_height += contours[i].y;
    }
    aver_height /= contours.size();
    return aver_height;
}

void pairingContours(vector<Point> contours, vector<Point> &farthestpair, vector<Point> &closestpair, Mat &rawimg)
{

    farthestpair.clear();
    closestpair.clear();
    if (contours.size() < 4)
    {
        cout << "contours size is less than 4" << endl;
        return;
    }

    Rect rect = boundingRect(contours);

    vector<Point> upcontours;
    vector<Point> downcontours;
    float aver_up_y = 0;
    float aver_down_y = 0;

    for (int i = 0; i < contours.size(); i++)
    {
        if (abs(contours[i].y - rect.y - rect.height / 2) < 20)
        {
            contours.erase(contours.begin() + i); // 删除接近中心的点
            i--;
            continue;
        }
        if (contours[i].y < rect.y + rect.height / 2 && contours[i].x > 10 && contours[i].y > 10&&contours[i].x<rawimg.cols-10)
        {
            upcontours.push_back(contours[i]); // 记录上边沿的点
        }
        else if (contours[i].y > rect.y + rect.height / 2)
        {
            downcontours.push_back(contours[i]); // 记录下边沿的点
        }
    }
    // cout << "upcontours size: " << upcontours.size() << endl;
    // cout << "downcontours size: " << downcontours.size() << endl;
    if (upcontours.size() == 0 || downcontours.size() == 0)
    {
        cout << "upcontours size or downcontours size is 0" << endl;
        return;
    }
    aver_up_y = findaverheight(upcontours);
    aver_down_y = findaverheight(downcontours);

    // drawContours(rawimg, upcontours, -1, Scalar(0, 0, 255), 2);
    // drawContours(rawimg, downcontours, -1, Scalar(0, 0, 255), 2);

    // for (int i = 0; i < downcontours.size(); i++)
    // {
    //     if (fabsf(downcontours[i].y - aver_down_y) > 60)
    //     {
    //         cout << "downcontours: " << downcontours << endl;
    //         downcontours.erase(downcontours.begin() + i);
    //         cout << "aver_down_y: " << aver_down_y << endl;
    //         cout << "erase downcontours: " << downcontours[i] << endl;
    //         i--;
    //     }
    // }

    // for (int i = 0; i < upcontours.size(); i++)
    // {
    //     if (fabsf(upcontours[i].y - aver_up_y) > 60)
    //     {
    //         cout << "upcontours: " << upcontours << endl;
    //         upcontours.erase(upcontours.begin() + i);
    //         cout << "aver_up_y: " << aver_up_y << endl;
    //         cout << "erase upcontours: " << upcontours[i] << endl;
    //         i--;
    //     }
    // }

    if (upcontours.size() == 0 || downcontours.size() == 0)
    {
        cout << "upcontours size or downcontours size is 0" << endl;
        return;
    }

    aver_down_y = findaverheight(downcontours);
    aver_up_y = findaverheight(upcontours);

    Point heightestupcontour = upcontours[0];
    Point heightestdowncontour = downcontours[0];
    Point lowestupcontour = upcontours[0];
    Point lowestdowncontour = downcontours[0];
    Point middleupcontour = upcontours[0];
    Point middledowncontour = downcontours[0];

    if (upcontours.size() != 0)
    {
        heightestupcontour = findheighestPoint(upcontours);
        lowestupcontour = findlowestPoint(upcontours);
    }
    if (downcontours.size() != 0)
    {
        heightestdowncontour = findheighestPoint(downcontours);
        lowestdowncontour = findlowestPoint(downcontours);
    }

    for (int i = 0; i < downcontours.size(); i++)
    {
        if (abs(downcontours[i].x - heightestupcontour.x) < 20)
        {
            // if (abs(downcontours[i].y - aver_down_y) < 10)
            // {
            downcontours[i].x = heightestupcontour.x;
            farthestpair.push_back(heightestupcontour);
            farthestpair.push_back(downcontours[i]);
            downcontours.erase(downcontours.begin() + i);
            i--;
            // cout << "farthestpair is " << farthestpair << endl;
            // }
        }

        if (abs(downcontours[i].x - lowestupcontour.x) < 5)
        {
            // if (abs(downcontours[i].y - aver_down_y) < 10)
            // {
            closestpair.push_back(lowestupcontour);
            closestpair.push_back(downcontours[i]);
            downcontours.erase(downcontours.begin() + i);
            i--;
            // cout << "closestpair is " << closestpair << endl;
            // }
        }
    }

    if ((farthestpair.empty() || (closestpair.empty())))
    {
        for (int i = 0; i < upcontours.size(); i++)
        {
            for (int j = 0; j < downcontours.size(); j++)
            {
                if (abs(upcontours[i].x - downcontours[j].x) < 5)
                {
                    // if ((abs(upcontours[i].y - aver_up_y) < 20) && (abs(downcontours[j].y - aver_down_y) < 20))
                    // {
                    if (farthestpair.empty())
                    {
                        farthestpair.push_back(upcontours[i]);
                        farthestpair.push_back(downcontours[j]);
                    }
                    else if (closestpair.empty())
                    {
                        closestpair.push_back(upcontours[i]);
                        closestpair.push_back(downcontours[j]);
                    }
                    // }
                }
            }
        }
    }
}

float getdepth(Mat depimg, Point tarpoint, int size)
{
    Rect rect(MIN(MAX(1, tarpoint.x - size / 2), depimg.cols - size - 1), MIN(MAX(1, tarpoint.y - size / 2), depimg.rows - size - 1), size, size);
    Mat roi = depimg(rect);
    // cout << "roi is " << roi << endl;
    float minValue = numeric_limits<double>::max();
    // cout << "debug9" << endl;
    for (int i = 0; i < roi.rows; i++)
    {
        for (int j = 0; j < roi.cols; j++)
        {
            float value = roi.at<float>(i, j);
            if (isnan(value) || isinf(value))
            {
                continue;
            }
            else if (value > 0 && value < minValue)
            {
                minValue = value;
            }
        }
    }

    float depth = minValue;
    // cout << "debug10" << endl;
    return depth;
}

float cubemeasure(Mat rawimg, Mat depimg, float cublen, NF *nf)
{
    if (isinf(cublen) || isnan(cublen))
    {
        return 0;
    }

    // rawimg.resize(640, 480);
    // imshow("rawimg", rawimg);
    // waitKey(1);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<Point> farthestpair;
    vector<Point> closestpair;
    float depth_farpair[2] = {0};
    float depth_closepair[2] = {0};
    Mat threshimg;
    Point3f pixel2camera_point_farpair[2];
    Point3f pixel2camera_point_closepair[2];
    float dist[2] = {0};

    namedWindow("threshimg", WINDOW_NORMAL);
    createTrackbar("thresh_max", "threshimg", 0, 255, NULL);
    createTrackbar("thresh_min", "threshimg", 0, 255, NULL);
    createTrackbar("thresh_green_min", "threshimg", 0, 255, NULL);
    createTrackbar("thresh_blue_green_max", "threshimg", 0, 255, NULL);
    int thresh_max = getTrackbarPos("thresh_max", "threshimg");
    int thresh_min = getTrackbarPos("thresh_min", "threshimg");
    int thresh_green_min = getTrackbarPos("thresh_green_min", "threshimg");
    int thresh_blue_green_max = getTrackbarPos("thresh_blue_green_max", "threshimg");
    if (thresh_max == 0)
    {
        setTrackbarPos("thresh_max", "threshimg", 120);
    }
    if (thresh_min == 0)
    {
        setTrackbarPos("thresh_min", "threshimg", 50);
    }
    if (thresh_green_min == 0)
    {
        setTrackbarPos("thresh_green_min", "threshimg", 100);
    }
    if (thresh_blue_green_max == 0)
    {
        setTrackbarPos("thresh_blue_green_max", "threshimg", 150);
    }

    predispose(rawimg, threshimg, 3, thresh_max, thresh_min, thresh_green_min, thresh_blue_green_max);

    findContours(threshimg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    if (contours.empty())
    {
        cout << "contours empty" << endl;
        return cublen;
    }

    else
    {
        vector<Point> maxsizeContours;
        findmaxsizeContours(contours, maxsizeContours);
        if (maxsizeContours.size() < 1000)
        {
            cout << "contours too small" << endl;
            return cublen;
        }

        vector<Point> target_contours;
        target_contours = contoursfilter(contours);

        // cout << "debug" << endl;

        pairingContours(target_contours, farthestpair, closestpair, rawimg);
        // cout << "farthestpair is " << farthestpair << endl;
        // cout << "closestpair is " << closestpair << endl;
        if (!farthestpair.empty())
        {

            depth_farpair[0] = getdepth(depimg, farthestpair[0], 4);
            depth_farpair[1] = getdepth(depimg, farthestpair[1], 4);
            // cout << "depth1:" << depth_farpair[0] << endl;
            // cout << "depth2:" << depth_farpair[1] << endl;
            if (isnan(depth_farpair[0]) || isnan(depth_farpair[1]) || isinf(depth_farpair[0]) || isinf(depth_farpair[1]) || fabsf(depth_farpair[0] - depth_farpair[1]) > 0.2)
            {
                cout << "farpair depth error" << endl;
            }
            else
            {
                pixel2camera_point_farpair[0] = pixel2camera(farthestpair[0], depth_farpair[0], camerargbinfo);
                pixel2camera_point_farpair[1] = pixel2camera(farthestpair[1], depth_farpair[1], camerargbinfo);
                // cout << "far1:" << pixel2camera_point_farpair[0] << endl;
                // cout << "far2:" << pixel2camera_point_farpair[1] << endl;
                dist[0] = norm(pixel2camera_point_farpair[0] - pixel2camera_point_farpair[1]);
                putText(rawimg, to_string(dist[0]), (farthestpair[0] + farthestpair[1]) / 2, FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);
                line(rawimg, farthestpair[0], farthestpair[1], Scalar(255, 255, 255), 2);
            }
        }
        if (!closestpair.empty())
        {

            depth_closepair[0] = getdepth(depimg, closestpair[0], 4);
            depth_closepair[1] = getdepth(depimg, closestpair[1], 4);
            // cout << "depth1:" << depth_closepair[0] << endl;
            // cout << "depth2:" << depth_closepair[1] << endl;
            if (isnan(depth_closepair[0]) || isnan(depth_closepair[1]) || isinf(depth_closepair[0]) || isinf(depth_closepair[1]) || fabsf(depth_closepair[0] - depth_closepair[1]) > 0.2)
            {
                cout << "closepair depth error" << endl;
                // return cublen;
            }
            else
            {
                pixel2camera_point_closepair[0] = pixel2camera(closestpair[0], depth_closepair[0], camerargbinfo);
                pixel2camera_point_closepair[1] = pixel2camera(closestpair[1], depth_closepair[1], camerargbinfo);
                // cout << "close1:" << pixel2camera_point_closepair[0] << endl;
                // cout << "close2:" << pixel2camera_point_closepair[1] << endl;
                dist[1] = norm(pixel2camera_point_closepair[0] - pixel2camera_point_closepair[1]);
                putText(rawimg, to_string(dist[1]), (closestpair[0] + closestpair[1]) / 2, FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);
                line(rawimg, closestpair[0], closestpair[1], Scalar(255, 255, 255), 2);
            }
        }

        vector<KeyPoint> keypoints;
        for (int i = 0; i < farthestpair.size(); i++)
        {
            keypoints.push_back(KeyPoint(farthestpair[i], 1));
            drawKeypoints(rawimg, keypoints, rawimg, Scalar(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            putText(rawimg, to_string(farthestpair[i].x) + "," + to_string(farthestpair[i].y) + "dep: " + to_string(depth_farpair[i]), farthestpair[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
        }

        for (int i = 0; i < closestpair.size(); i++)
        {
            keypoints.push_back(KeyPoint(closestpair[i], 1));
            drawKeypoints(rawimg, keypoints, rawimg, Scalar(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            putText(rawimg, to_string(closestpair[i].x) + "," + to_string(closestpair[i].y) + "dep: " + to_string(depth_closepair[i]), closestpair[i], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
        }

        // cout << "debug1" << endl;
        if (farthestpair.empty() && closestpair.empty())
        {
            cout << "no pair" << endl;
            return cublen;
        }
        else if (farthestpair.empty())
        {
            cout << "no farthestpair" << endl;
        }
        else if (closestpair.empty())
        {
            cout << "no closestpair" << endl;
        }

        cublen = getcubelength(dist, cublen, nf);
        cout << "cublen is " << cublen << endl;
        // if (nf->E_est_n_1 <= 0.001)
        // {
        //     imwrite(getpath(outputimg_path, "cubemeasure"), rawimg);
        // }

        if (nf->E_est_n_1 <= 0.005 && fabsf(dist[0] - dist[1]) < 0.05)
        {
            imwrite(getpath(outputimg_path, "cubemeasure"), rawimg);
            cout << "img saved" << endl;
            ros::NodeHandle nh;
            ros::Publisher pub = nh.advertise<std_msgs::String>("cubemeasure_result_str", 1);
            std_msgs::String msg;
            msg.data = to_string(cublen);
            pub.publish(msg);
        }

        return fabsf(cublen);
    }
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
    sub_rgb = nh.subscribe("/camera2/rgb/image_raw", 1, imageCallback);
    sub_depth = nh.subscribe("/camera2/depth/image_raw", 1, depthCallback);
    sub_camerargbinfo = nh.subscribe("/camera2/depth/camera_info", 1, camerargbinfoCallback);
    ros::Publisher pub = nh.advertise<std_msgs::String>("cubemeasure_result_str", 1);
    ros::Rate loop_rate(10);
    namedWindow("cubemeasure", WINDOW_AUTOSIZE);
    startWindowThread();
    while (ros::ok())
    {
        ros::spinOnce();
        if (raw_img.empty() || raw_depth.empty() || camerargbinfo.empty())
        {
            continue;
        }
        cubelength = cubemeasure(raw_img, raw_depth, cubelength, &nf);
        imshow("cubemeasure", raw_img);
        // std_msgs::String msg;
        // std::stringstream ss;
        // ss << "length";
        // msg.data = ss.str();
        // pub.publish(msg);
        loop_rate.sleep();
    }

    return 0;
}