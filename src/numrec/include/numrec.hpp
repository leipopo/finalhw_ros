#ifndef NUMREC_HPP
#define NUMREC_HPP
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
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include "pos_pub.hpp"

#define img_path "/img"
#define traincfg_path "/traindata/traincfg.txt"
#define svm_path_result "/traindata/svm_result.txt"
#define knn_path_result "/traindata/knn_result.xml"

using namespace cv;
using namespace std;

typedef struct
{
    Mat img;
    string result_str;
} numrec_result;

#define thresh_value 2

#define imgx 60
#define imgy 70

#define Kvalue 3

#endif 