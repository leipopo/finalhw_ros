#include "numrec.hpp"

numrec_result num_rec(Mat raw_tarimg)
{
    numrec_result result;
    // Mat gray_tarimg = imread(sampleimg_path, IMREAD_GRAYSCALE);
    // imshow("gray_tarimg", gray_tarimg);

    Mat gray_tarimg;
    cvtColor(raw_tarimg, gray_tarimg, COLOR_BGR2GRAY); // 灰度图

    Mat blur_tarimg;
    Ptr<ml::KNearest> knn = ml::KNearest::create();
    knn = ml::KNearest::load(getpath(knn_path_result, "numrec").c_str());
    blur(gray_tarimg, blur_tarimg, Size(3, 3)); // 模糊
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
    // } //测量阈值
    threshold(blur_tarimg, thresh_tarimg, thresh_value, 255, THRESH_BINARY_INV); // 二值化
    // imshow("thresh_tarimg", thresh_tarimg);

    vector<vector<Point>> contours;                                                                    // 轮廓
    vector<Vec4i> hierarchy;                                                                           // 轮廓的结构信息
    findContours(thresh_tarimg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0)); // 查找轮廓

    int result_num_position[contours.size()][2];
    for (int i = 0; i < contours.size(); i++)
    {

        if (contourArea(contours[i]) > 300 && contourArea(contours[i]) < 3000)
        {
            // cout <<"area: "<< contourArea(contours[i]) << endl;
            Rect rect = boundingRect(contours[i]);
            rectangle(raw_tarimg, rect, Scalar(255, 0, 255), 2, 8, 0); // 画矩形
            // imshow("thresh_tarimg", result_img);

            Mat roi = thresh_tarimg(rect);
            Mat roi_resized;
            resize(roi, roi_resized, Size(imgx, imgy), 0, 0, INTER_LINEAR); // 重置大小
            // imshow("roi", roi_resized);

            Mat fea = Mat::zeros(1, imgx * imgy, CV_32FC1);
            fea = roi_resized.reshape(1, 1);
            fea.convertTo(fea, CV_32FC1);

            int result_num = knn->predict(fea);

            result_num_position[i][0] = knn->predict(fea);
            result_num_position[i][1] = rect.x;

            putText(raw_tarimg, to_string(result_num_position[i][0]), Point(rect.x, rect.y - 5), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(255, 0, 255), 2, 8, 0);
            // cout << "num: " << result_num_position[i][0] << endl;
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
Mat raw_tarimg;
static const char WINDOW[] = "numrec";

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{

    raw_tarimg = cv_bridge::toCvShare(msg, "bgr8")->image;
    numrec_result result = num_rec(raw_tarimg);
    // cout << "result_str: " << result.result_str << endl;
    imshow(WINDOW, result.img);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("numrec_result_str", 1);

    // cout << "result.result_str.size: " << result.result_str.size() << endl;
    if (result.result_str.size() == 4)
    {
        int result_error_flag = 0;
        // cout << "result.result_str: " << result.result_str << endl;
        for (int i = 0; i < 4; i++)
        {
        //     cout << "i: " << i << endl;
        //     cout << "result.result_str[i]: " << result.result_str[i] << endl;
            for (int j = i+1; j < 4; j++)
            {
                if (result.result_str[i] == result.result_str[j])
                {
                    cout << "result_error" << endl;
                    result_error_flag = 1;
                    return;
                }
            }
        }
        std_msgs::String result_str;
        result_str.data = result.result_str;
        cout << "result_str.data: " << result_str.data << endl;
        pub.publish(result_str);
    }
    else
    {
        cout << "result_error" << endl;
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "numrec");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/camera2/rgb/image_raw", 1, imageCallback);
    ros::Publisher pub = nh.advertise<std_msgs::String>("numrec_result_str", 1);

    namedWindow(WINDOW);
    startWindowThread();
    ros::spin();
    return 0;
}
