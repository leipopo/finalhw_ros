#include "pos_pub.hpp"

string record_path;

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
    cout<<"writeto:"<<record_path<<endl;
    writefile(record_path.c_str(), date);
    ROS_INFO("date:%s", date.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_goal_record");
    ros::NodeHandle n;

    if (argv[1][0] == '1')
    {
        record_path = getpath(record_path1, "pospub");
        ROS_INFO("recording_1");
    }
    else if (argv[1][0] == '2')
    {
        record_path = getpath(record_path2, "pospub");
        ROS_INFO("recording_2");
    }
    else if (argv[1][0] == '3')
    {
        record_path = getpath(record_path3, "pospub");
        ROS_INFO("recording_3");
    }
    else if (argv[1][0] == '4')
    {
        record_path = getpath(record_path4, "pospub");
        ROS_INFO("recording_4");
    }
    else
    {
        ROS_INFO("wrong input");
        return 0;
    }
    deletefile(record_path.c_str());

    ros::Subscriber rec_sub = n.subscribe("/move_base_simple/goal", 25, callback_path);

    ros::spin();
    return 0;
}