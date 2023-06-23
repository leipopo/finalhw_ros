#include "pos_pub.hpp"

float position[100][7];
int line_num_read = 0, line_num_send = 0;
int race_path_line_num = 0;

int tarpath = 0;

int flag_begin = 0;
int flag_num = 0;
int flag_cube = 0;
int flag_time = 0;

/*
 *功能：读取以私有格式记录的路径
 *输入1：路径文件路径
 *输入2：缓存数组
 *输入3：已读取的行数
 *输出：已读取的行数
 */
int read_pathrecord(const char *path, float position[][7], int linenumread)
{
    fstream file;
    string line;
    int begin, end;

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

ros::Subscriber sub_result;
ros::Subscriber sub_numrec;
ros::Subscriber sub_cube;
ros::Subscriber sub_globalplan;

time_t begin_time, end_time;
ros::Time ros_begin_time, ros_end_time;

/*
 *功能：处理/move_base/GlobalPlanner/plan话题的回调函数，根据是否规划来判断是否直接跳过当前导航点发布下一个导航点
 */
void globalplanCallback(const nav_msgs::Path::ConstPtr &msg)
{
    if (msg->poses.size() == 0)
    {
        // ROS_INFO("globalplanCallback: NULL");
        line_num_send++;
    }
    // else
    // {
    //     ROS_INFO("globalplanCallback: %d", msg->poses.size());
    // }
}

/*
 *功能：处理/move_base/result话题的回调函数，判断是否到达导航点，到达则判断是否到达最后一个导航点，到达则结束或者根据标志位读取新的路径，否则发布下一个导航点
 */
void resultCallback(const move_base_msgs::MoveBaseActionResult &msg)
{
    // ROS_INFO("result: %d", msg.status.status);

    flag_begin = 1;

    if (flag_time == 0) // 打印起始时间
    {
        ros_begin_time = ros::Time::now();
        cout << "ros_begin_time: " << ros_begin_time.toSec() << endl;
        begin_time = time(nullptr);
        string time = ctime(&begin_time);
        cout << "begin_time: " << time << endl;
        flag_time = 1;
    }
    if (flag_time == 2) // 打印结束时间
    {
        ros_end_time = ros::Time::now();
        cout << "ros_end_time: " << ros_end_time.toSec() << endl;
        end_time = time(nullptr);
        string time = ctime(&end_time);
        cout << "end_time: " << time << endl;
        string cost_time = to_string(end_time - begin_time);
        cout << "cost_time: " << cost_time << endl;
        ros::Duration ros_cost_time = ros_end_time - ros_begin_time;
        cout << "ros_cost_time: " << ros_cost_time.toSec() << endl;
        flag_time = 3;
    }

    if (msg.status.status == 3) // 成功到达导航点
    {
        ros::NodeHandle nh;
        geometry_msgs::PoseStamped goal;

        // 向缓存写入下一个导航点
        goal.header.frame_id = "odom";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = position[line_num_send][0];
        goal.pose.position.y = position[line_num_send][1];
        goal.pose.position.z = position[line_num_send][2];
        goal.pose.orientation.x = position[line_num_send][3];
        goal.pose.orientation.y = position[line_num_send][4];
        goal.pose.orientation.z = position[line_num_send][5];
        goal.pose.orientation.w = position[line_num_send][6];

        if (!flag_num) // 判断是否识别到数字，未识别到就执行录制路径的最后两个点朝数字方向扫描
        {
            if (line_num_send < race_path_line_num - 1)
            {
                if (line_num_send == race_path_line_num - 2 && flag_time == 1)
                {

                    flag_time = 2;
                }
                line_num_send++;
            }

            else if (line_num_send == race_path_line_num - 1)
            {

                line_num_send = race_path_line_num - 2;
            }
        }
        else
        {
            line_num_send++; // 由于line_num_send直接作为数组下标需要从零开始，而line_num_read从1开始，所以line_num_send后＋1与line_num_read进行比对
        }

        if (line_num_send <= line_num_read) // 到达最后一个点及之前的点
        {
            // ROS_INFO("goal: %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
            // ROS_INFO("goal: %f, %f, %f, %f", goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);
            // cout << "goal" << goal.pose << endl;
            ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
            pub_goal.publish(goal);
            if (line_num_read == line_num_send)
            {
                if (flag_cube == 1 && flag_num == 1)
                {
                    sub_result.shutdown();
                    sub_numrec.shutdown();
                    sub_globalplan.shutdown();
                    sub_cube.shutdown();
                    ros::shutdown();
                    cout << "finish" << endl;
                    return;
                }
                else if (flag_cube == 0 && flag_num == 1)
                {
                    line_num_read = read_pathrecord(getpath(record_path_end, "pospub").c_str(), position, line_num_read);
                }
            }
        }
    }
    if (flag_cube == 1 && flag_num == 1)
    {
        line_num_send = line_num_read - 1;
        // cout << "finish" << endl;
    }
    // cout << "flag_num: " << flag_num << endl;
    // cout << "flag_cube: " << flag_cube << endl;
    // cout << "line_num_read: " << line_num_read << endl;
    // cout << "line_num_send: " << line_num_send << endl;
}

/**
 *功能：接收数字识别结果,比对数字与目标数字是否一致，一致则flag_num置1，并读取对应路径及结束路径
 */
void numrecCallback(const std_msgs::String &msg)
{
    string num = msg.data;

    if (flag_num == 0)
    {
        if (num.size() == 4)
        {
            for (int i = 0; i < 4; i++)
            {
                num[i] = num[i] - '0';
                if (num[i] == tarpath)
                {
                    flag_num = 1;
                    if (i == 0)
                    {
                        line_num_read = read_pathrecord(getpath(record_path1, "pospub").c_str(), position, line_num_read);
                        // cout << "i: " << i << endl;
                        // cout << "read_path1" << endl;
                    }
                    else if (i == 1)
                    {
                        line_num_read = read_pathrecord(getpath(record_path2, "pospub").c_str(), position, line_num_read);
                        // cout << "i: " << i << endl;
                        // cout << "read_path2" << endl;
                    }
                    else if (i == 2)
                    {
                        line_num_read = read_pathrecord(getpath(record_path3, "pospub").c_str(), position, line_num_read);
                        // cout << "i: " << i << endl;
                        // cout << "read_path3" << endl;
                    }
                    else if (i == 3)
                    {
                        line_num_read = read_pathrecord(getpath(record_path4, "pospub").c_str(), position, line_num_read);
                        // cout << "i: " << i << endl;
                        // cout << "read_path4" << endl;
                    }
                    sub_numrec.shutdown();
                    line_num_read = read_pathrecord(getpath(record_path_end, "pospub").c_str(), position, line_num_read);
                    // cout << "read_path_end" << endl;
                    break;
                }
            }
        }
    }
}

/**
 *功能：接收正方体边长测量结果，是则flag_cube置1
 */
void cubemeasureCallback(const std_msgs::String &msg)
{
    // cout << "cube_str: " << msg.data << endl;
    // cout << "cube_float" << stof(msg.data) << endl;
    // cout << "flag_cube: " << flag_cube << endl;
    // cout << "flag_num: " << flag_num << endl;
    if (stof(msg.data) >= 0.98 && stof(msg.data) <= 2.02 && flag_cube == 0 && flag_num == 1)
    {
        flag_cube = 1;
        // sub_cube.shutdown();
    }
}

int main(int argc, char **argv)
{

    tarpath = argv[1][0] - '0';

    ros::init(argc, argv, "move_base_goal_pub");
    ros::NodeHandle nh;

    line_num_read = read_pathrecord(getpath(record_path_begin, "pospub").c_str(), position, line_num_read);
    race_path_line_num = line_num_read;

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
    sub_result = nh.subscribe("/move_base/result", 1000, &resultCallback);
    sub_numrec = nh.subscribe("/numrec_result_str", 1000, &numrecCallback);
    sub_cube = nh.subscribe("/cubemeasure_result_str", 1000, &cubemeasureCallback);
    sub_globalplan = nh.subscribe("/move_base/GlobalPlanner/plan", 1000, &globalplanCallback);
    ros::Rate loop_rate(500);
    while (!flag_begin)
    {
        pub_goal.publish(goal);
        // ROS_INFO("goal: %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
        // ROS_INFO("goal: %f, %f, %f, %f", goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
}
