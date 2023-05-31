#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>


using namespace std;

#define record_path "/home/lpga/finalhw_ros/src/urdf_gazebo_demo/pathrec/pathrecordend.txt"

void writefile(const char *path, string data)
{
    ofstream file;
    file.open(path, ios::out | ios::app);
    // ROS_INFO("file opened");
    file << data;
    // ROS_INFO("data written");
    file.close();
}

string readfile(const char *path)
{
    ifstream file;
    file.open(path, ios::in);
    string data;
    file >> data;
    file.close();
    return data;
}
void deletefile(const char *path)
{
    remove(path);
}

void string2char(string str, char *ch)
{
    int i;
    for (i = 0; i < str.length(); i++)
    {
        ch[i] = str[i];
    }
    ch[i] = '\0';
}

void callback_path(const geometry_msgs::PoseStamped &msg)
{
    // outfile用法同cout,存储形式 1 2 3
    string date;
    date+="x=";
    date+= to_string(msg.pose.position.x);
    date+="; ";
    date+="y=";
    date+= to_string(msg.pose.position.y);
    date+="; ";
    date+="z=";
    date+= to_string(msg.pose.position.z);
    date+="; ";
    date+="qx=";
    date+= to_string(msg.pose.orientation.x );
    date+="; ";
    date+="qy=";
    date+= to_string(msg.pose.orientation.y);
    date+="; ";
    date+="qz=";
    date+= to_string(msg.pose.orientation.z);
    date+="; ";
    date+="qw=";
    date+= to_string(msg.pose.orientation.w);
    date+="\n";
    writefile(record_path, date);
    ROS_INFO("date:%s",date.c_str());

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_goal_record");
    ros::NodeHandle n;

    deletefile(record_path);

    ros::Subscriber rec_sub = n.subscribe("/move_base_simple/goal", 25, callback_path);
    ROS_INFO("recording_end");

    ros::spin();
    return 0;
}