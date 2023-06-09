#include "numrec.hpp"

numrec_result num_rec(Mat raw_tarimg)
{
    numrec_result result;
    // Mat gray_tarimg = imread(sampleimg_path, IMREAD_GRAYSCALE);
    // imshow("gray_tarimg", gray_tarimg);

    Mat gray_tarimg;
    cvtColor(raw_tarimg, gray_tarimg, COLOR_BGR2GRAY);

    Mat blur_tarimg;
    Ptr<ml::KNearest> knn = ml::KNearest::create();
    knn = ml::KNearest::load(knn_path_result);
    blur(gray_tarimg, blur_tarimg, Size(3, 3));
    // imshow("blur_tarimg", blur_tarimg);

    Mat thresh_tarimg;
    // namedWindow("thresh_tarimg", WINDOW_NORMAL);
    // createTrackbar("thresh_tarimg", "thresh_tarimg", 0, 255);
    // while (1)
    // {
    //     int thresh_tarimg_value = getTrackbarPos("thresh_tarimg", "thresh_tarimg");
    //     threshold(blur_tarimg, thresh_tarimg, thresh_tarimg_value, 255, THRESH_BINARY_INV);
    //     imshow("thresh_tarimg", thresh_tarimg);
    //     waitKey(1);
    // }

    threshold(blur_tarimg, thresh_tarimg, thresh_value, 255, THRESH_BINARY_INV);
    // imshow("thresh_tarimg", thresh_tarimg);

    vector<vector<Point>> contours;                                                                    // 轮廓
    vector<Vec4i> hierarchy;                                                                           // 轮廓的结构信息
    findContours(thresh_tarimg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0)); // 查找轮廓

    // Mat result_img = Mat::zeros(thresh_tarimg.size(), CV_8UC3);
    // result_img = thresh_tarimg.clone();
    // cout << contours.size() << endl;
    int result_num_position[contours.size()][2];
    for (int i = 0; i < contours.size(); i++)
    {

        // cout << contourArea(contours[i]) << endl;
        if (contourArea(contours[i]) > 300)
        {

            Rect rect = boundingRect(contours[i]);
            rectangle(raw_tarimg, rect, Scalar(255, 0, 255), 2, 8, 0);
            // imshow("thresh_tarimg", result_img);

            Mat roi = thresh_tarimg(rect);
            Mat roi_resized;
            resize(roi, roi_resized, Size(imgx, imgy), 0, 0, INTER_LINEAR);
            // imshow("roi", roi_resized);

            Mat fea = Mat::zeros(1, imgx*imgy, CV_32FC1);
            fea = roi_resized.reshape(1, 1);
            fea.convertTo(fea, CV_32FC1);

            // cout<<fea<<endl;

            result_num_position[i][0] = knn->predict(fea);
            result_num_position[i][1] = rect.x;

            putText(raw_tarimg, to_string(result_num_position[i][0]), Point(rect.x, rect.y-5), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(255, 0, 255), 2, 8, 0);

        }
        else
        {
            result_num_position[i][0] = 0;
            result_num_position[i][1] = 0;
        }
    }

    for (int i = 0; i < contours.size(); i++)
    {
        for (int j = 0; j < contours.size() - 1; j++)
        {
            if (result_num_position[j][1] > result_num_position[j + 1][1])
            {
                int temp[2] = {0, 0};
                temp[0] = result_num_position[j][0];
                result_num_position[j][0] = result_num_position[j + 1][0];
                result_num_position[j + 1][0] = temp[0];
                temp[1] = result_num_position[j][1];
                result_num_position[j][1] = result_num_position[j + 1][1];
                result_num_position[j + 1][1] = temp[1];
            }
        }
    }

    for (int i = 0; i < contours.size(); i++)
    {
        if (result_num_position[i][0] != 0)
        {
            if (i > 0)
            {
                if (result_num_position[i][0] != result_num_position[i - 1][0])
                {
                    result.result_str += to_string(result_num_position[i][0]);
                }
            }
            else
            {
                result.result_str += to_string(result_num_position[i][0]);
            }
        }
    }

    result.img = raw_tarimg;
    return result;
}

// int main(int argc, char **argv)
// {
//     string sampleimg_path = "/home/lpga/finalhw_ros/src/numrec/img/t1-4.jpg";
//     Mat gray_tarimg = imread(sampleimg_path, IMREAD_GRAYSCALE);
//     numrec_result result = num_rec(gray_tarimg);
//     imshow("result_img", result.img);
//     cout << "result_str: " << result.result_str << endl;

//     waitKey(0);
//     return 0;

// }

numrec_result result;
static const char WINDOW[] = "Image window";

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        Mat raw_tarimg = cv_bridge::toCvShare(msg, "bgr8")->image;
        numrec_result result = num_rec(raw_tarimg);
        imshow(WINDOW, result.img);
        cout << "result_str: " << result.result_str << endl;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to ''.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_goal_pub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera/image_raw", 1, imageCallback);
    namedWindow(WINDOW);
    startWindowThread();
    ros::spin();

    return 0;
}