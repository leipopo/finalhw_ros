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

#define img_path "/home/lpga/finalhw_ros/src/numrec/img"
#define traincfg_path "/home/lpga/finalhw_ros/src/numrec/traindata/traincfg.txt"
#define svm_path_result "/home/lpga/finalhw_ros/src/numrec/traindata/svm_result.txt"
#define knn_path_result "/home/lpga/finalhw_ros/src/numrec/traindata/knn_result.xml"

using namespace cv;
using namespace std;

typedef struct
{
    Mat img;
    string result_str;
} numrec_result;

int thresh_value = 4;
