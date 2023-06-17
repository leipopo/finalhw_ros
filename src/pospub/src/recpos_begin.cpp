#include "pos_pub.hpp"

void callback_path(const geometry_msgs::PoseStamped &msg)
{
    // outfile用法同cout,存储形式 1 2 3
    string date;
    date += "x=";
    date += to_string(msg.pose.position.x);
    date += "; ";
    date += "y=";
    date += to_string(msg.pose.position.y);
    date += "; ";
    date += "z=";
    date += to_string(msg.pose.position.z);
    date += "; ";
    date += "qx=";
    date += to_string(msg.pose.orientation.x);
    date += "; ";
    date += "qy=";
    date += to_string(msg.pose.orientation.y);
    date += "; ";
    date += "qz=";
    date += to_string(msg.pose.orientation.z);
    date += "; ";
    date += "qw=";
    date += to_string(msg.pose.orientation.w);
    date += "\n";
    writefile(getpath(record_path_begin,"pospub").c_str(), date);
    ROS_INFO("date:%s", date.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_goal_record");
    ros::NodeHandle n;

    deletefile(getpath(record_path_begin,"pospub").c_str());

    ros::Subscriber rec_sub = n.subscribe("/move_base_simple/goal", 25, callback_path);
    ROS_INFO("recording_begin");

    ros::spin();
    return 0;
}