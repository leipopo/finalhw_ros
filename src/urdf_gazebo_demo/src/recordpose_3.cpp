#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <ctime>

using namespace std;

void callback_path(const geometry_msgs::PoseStamped &msg)
{
    // outfile用法同cout,存储形式 1 2 3
    ofstream outfile;
    outfile.setf(ios::fixed, ios::floatfield);
    outfile.precision(2);
    outfile.open("/home/lpga/demo01_ws/src/urdf_gazebo_demo/pathrec/pathrecord3.txt", std::ios::app);
    outfile << msg.pose.position.x << " " << msg.pose.position.y << endl;
    ROS_INFO("recorded_3");
    outfile.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_goal_record");
    ros::NodeHandle n;

    ros::Subscriber rec_sub = n.subscribe("/move_base_simple/goal", 25, callback_path);
    ROS_INFO("recording_3");

    ros::spin();
    return 0;
}