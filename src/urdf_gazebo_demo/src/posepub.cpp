#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>

using namespace std;

#define record_path_begin "/home/lpga/finalhw_ros/src/urdf_gazebo_demo/pathrec/pathrecordbegin.txt"
#define record_path1 "/home/lpga/finalhw_ros/src/urdf_gazebo_demo/pathrec/pathrecord1.txt"
#define record_path2 "/home/lpga/finalhw_ros/src/urdf_gazebo_demo/pathrec/pathrecord2.txt"
#define record_path3 "/home/lpga/finalhw_ros/src/urdf_gazebo_demo/pathrec/pathrecord3.txt"
#define record_path4 "/home/lpga/finalhw_ros/src/urdf_gazebo_demo/pathrec/pathrecord4.txt"
#define record_path_end "/home/lpga/finalhw_ros/src/urdf_gazebo_demo/pathrec/pathrecordend.txt"

void resultCallback(const move_base_msgs::MoveBaseActionResult &msg);

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

float position[100][7];
int line_num_read = 0, line_num_send = 0;
int flag = 0;

int read_pathrecord(const char *path, float position[][7], int linenumread)
{
    fstream file;
    string line;
    int begin, end;
    int linenum = linenumread;

    file.open(path, ios::in);

    while (!file.eof())
    {
        getline(file, line);

        begin = line.find_first_of('=', 0);
        end = line.find_first_of(';', begin);
        string x = line.substr(begin + 1, end - begin - 1);
        // ROS_INFO("read: %s", x.c_str());
        // ROS_INFO("read: %d ", begin);
        // ROS_INFO("read: %d ", end);

        begin = line.find_first_of('=', end);
        end = line.find_first_of(';', begin);
        string y = line.substr(begin + 1, end - begin - 1);

        begin = line.find_first_of('=', end);
        end = line.find_first_of(';', begin);
        string z = line.substr(begin + 1, end - begin - 1);

        begin = line.find_first_of('=', end);
        end = line.find_first_of(';', begin);
        string qx = line.substr(begin + 1, end - begin - 1);

        begin = line.find_first_of('=', end);
        end = line.find_first_of(';', begin);
        string qy = line.substr(begin + 1, end - begin - 1);

        begin = line.find_first_of('=', end);
        end = line.find_first_of(';', begin);
        string qz = line.substr(begin + 1, end - begin - 1);

        begin = line.find_first_of('=', end);
        end = line.find_first_of(';', begin);
        string qw = line.substr(begin + 1, end - begin - 1);

        position[linenumread][0] = atof(x.c_str());
        position[linenumread][1] = atof(y.c_str());
        position[linenumread][2] = atof(z.c_str());
        position[linenumread][3] = atof(qx.c_str());
        position[linenumread][4] = atof(qy.c_str());
        position[linenumread][5] = atof(qz.c_str());
        position[linenumread][6] = atof(qw.c_str());

        linenumread++;
    }
    file.close();
    return linenumread;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_goal_pub");
    ros::NodeHandle nh;

    // line_num_read = read_pathrecord(record_path_begin, position, line_num_read);

    if (argv[1][0] == '1')
    {
        line_num_read = read_pathrecord(record_path1, position, line_num_read);
        ROS_INFO("cargothrough1");
    }
    else if (argv[1][0] == '2')
    {
        line_num_read = read_pathrecord(record_path2, position, line_num_read);
        ROS_INFO("cargothrough2");
    }
    else if (argv[1][0] == '3')
    {
        line_num_read = read_pathrecord(record_path3, position, line_num_read);
        ROS_INFO("cargothrough3");
    }
    else if (argv[1][0] == '4')
    {
        line_num_read = read_pathrecord(record_path4, position, line_num_read);
        ROS_INFO("cargothrough4");
    }
    else
    {
        ROS_INFO("wrong input");
    }
    line_num_read = read_pathrecord(record_path_end, position, line_num_read);

    line_num_send = 0;

    ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "odom";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = position[0][0];
    goal.pose.position.y = position[0][1];
    goal.pose.position.z = position[0][2];
    goal.pose.orientation.x = position[0][3];
    goal.pose.orientation.y = position[0][4];
    goal.pose.orientation.z = position[0][5];
    goal.pose.orientation.w = position[0][6];
    line_num_send++;
    ros::Subscriber sub_result = nh.subscribe("/move_base/result", 1000, &resultCallback);

    ros::Rate loop_rate(500);
    while (!flag)
    {
        pub_goal.publish(goal);
        ROS_INFO("goal: %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
        ROS_INFO("goal: %f, %f, %f, %f", goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
}

void resultCallback(const move_base_msgs::MoveBaseActionResult &msg)
{
    ROS_INFO("result: %d", msg.status.status);
    flag = 1;
    if (msg.status.status == 3)
    {
        ros::NodeHandle nh;
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "odom";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = position[line_num_send][0];
        goal.pose.position.y = position[line_num_send][1];
        goal.pose.position.z = position[line_num_send][2];
        goal.pose.orientation.x = position[line_num_send][3];
        goal.pose.orientation.y = position[line_num_send][4];
        goal.pose.orientation.z = position[line_num_send][5];
        goal.pose.orientation.w = position[line_num_send][6];
        line_num_send++;
        if (line_num_send <= line_num_read)
        {
            ROS_INFO("goal: %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
            ROS_INFO("goal: %f, %f, %f, %f", goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);
            ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
            pub_goal.publish(goal);
        }
    }
}
