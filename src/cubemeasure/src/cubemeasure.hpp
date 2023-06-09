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

using namespace cv;
using namespace std;