#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#include "opencv4/opencv2/ml.hpp"
#include "opencv4/opencv2/imgcodecs.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdio.h>
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace cv;
using namespace std;

#define img_path "/home/lpga/finalhw_ros/src/numrec/img"
#define traincfg_path "/home/lpga/finalhw_ros/src/numrec/traindata/traincfg.txt"
#define svm_path_result "/home/lpga/finalhw_ros/src/numrec/traindata/svm_result.txt"
#define knn_path_result "/home/lpga/finalhw_ros/src/numrec/traindata/knn_result.xml"

int main(int argc, const char **argv)
{
    string sampleimg_path = "/home/lpga/finalhw_ros/src/numrec/img/t1-4.jpg";
    Mat gray_tarimg = imread(sampleimg_path, IMREAD_GRAYSCALE);
    imshow("gray_tarimg", gray_tarimg);

    Mat blur_tarimg;
    blur(gray_tarimg, blur_tarimg, Size(3, 3));
    imshow("blur_tarimg", blur_tarimg);

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

    threshold(blur_tarimg, thresh_tarimg, 36, 255, THRESH_BINARY_INV);
    imshow("thresh_tarimg", thresh_tarimg);

    vector<vector<Point>> contours;                                                                    // 轮廓
    vector<Vec4i> hierarchy;                                                                           // 轮廓的结构信息
    findContours(thresh_tarimg, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0)); // 查找轮廓

    Mat result_img = Mat::zeros(thresh_tarimg.size(), CV_8UC3);
    result_img = thresh_tarimg.clone();
    // cout << contours.size() << endl;
    for (int i = 0; i < contours.size(); i++)
    {
        // cout << contourArea(contours[i]) << endl;
        if (contourArea(contours[i]) > 100)
        {

            Rect rect = boundingRect(contours[i]);
            rectangle(result_img, rect, Scalar(255, 255, 255), 1, 8, 0);
            imshow("thresh_tarimg", result_img);

            Mat roi = thresh_tarimg(rect);
            Mat roi_resized;
            resize(roi, roi_resized, Size(180, 210), 0, 0, INTER_LINEAR);
            imshow("roi", roi_resized);

            Mat fea = Mat::zeros(1, 37800, CV_32FC1);
            fea = roi_resized.reshape(1, 1);
            fea.convertTo(fea, CV_32FC1);
            Ptr<ml::KNearest> knn = ml::KNearest::create();
            knn = ml::KNearest::load(knn_path_result);
            // cout<<fea<<endl;

            int result = knn->predict(fea);
            cout << "result: " << result << endl;
            putText(result_img, to_string(result), Point(rect.x, rect.y), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 1, 8, 0);

            imshow("thresh_tarimg", result_img);
            waitKey(0);
        }
    }
    waitKey(0);
    return 0;
}