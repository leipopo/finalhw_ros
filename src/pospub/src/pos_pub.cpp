#include "pos_pub.hpp"

float position[100][7];
int line_num_read = 0, line_num_send = 0;
int race_path_line_num = 0;

int tarpath = 0;

int flag_begin = 0;
int flag_num = 0;
int flag_cube = 0;
int flag_time = 0;

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

void resultCallback(const move_base_msgs::MoveBaseActionResult &msg)
{
    // ROS_INFO("result: %d", msg.status.status);
    time_t begin_time, end_time;
    ros::Time ros_begin_time, ros_end_time;
    flag_begin = 1;

    if (flag_time == 0)
    {   
        ros_begin_time = ros::Time::now();
        cout << "ros_begin_time: " << ros_begin_time << endl;
        begin_time = time(nullptr);
        string time = ctime(&begin_time);
        cout << "begin_time: " << time << endl;
        flag_time = 1;
    }
    if (flag_time == 2)
    {
        ros_end_time = ros::Time::now();
        cout << "ros_end_time: " << ros_end_time << endl;
        end_time = time(nullptr);
        string time = ctime(&end_time);
        cout << "end_time: " << time << endl;
        string cost_time = to_string(end_time - begin_time);
        cout << "cost_time: " << cost_time << endl;
        cout << "ros_cost_time: " << ros_end_time - ros_begin_time << endl;
        flag_time = 3;
    }

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

        if (!flag_num)
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
            line_num_send++;
        }

        if (line_num_send <= line_num_read)
        {
            // ROS_INFO("goal: %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
            // ROS_INFO("goal: %f, %f, %f, %f", goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);
            ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
            pub_goal.publish(goal);
        }
        else if (flag_cube == 1)
        {
            ROS_INFO("finish");
            ros::shutdown();
        }
        else
        {
            line_num_read = read_pathrecord(getpath(record_path_end, "pospub").c_str(), position, line_num_read);
        }
    }
}

void numrecCallback(const std_msgs::String &msg)
{
    string num = msg.data;

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
                }
                else if (i == 1)
                {
                    line_num_read = read_pathrecord(getpath(record_path2, "pospub").c_str(), position, line_num_read);
                }
                else if (i == 2)
                {
                    line_num_read = read_pathrecord(getpath(record_path3, "pospub").c_str(), position, line_num_read);
                }
                else if (i == 3)
                {
                    line_num_read = read_pathrecord(getpath(record_path4, "pospub").c_str(), position, line_num_read);
                }
                sub_numrec.shutdown();
                break;
            }
        }
    }
}

void cubemeasureCallback(const std_msgs::String &msg)
{
    if (stof(msg.data) >= 0.9)
    {
        flag_cube = 1;
        sub_cube.shutdown();
    }
}

int main(int argc, char **argv)
{

    tarpath = argv[1][0] - '0';

    ros::init(argc, argv, "move_base_goal_pub");
    ros::NodeHandle nh;

    line_num_read = read_pathrecord(getpath(record_path_begin, "pospub").c_str(), position, line_num_read);
    race_path_line_num = line_num_read;

    // if (argv[1][0] == '1')
    // {
    //     line_num_read = read_pathrecord(getpath(record_path1, "pospub").c_str(), position, line_num_read);
    //     ROS_INFO("cargothrough1");
    // }
    // else if (argv[1][0] == '2')
    // {
    //     line_num_read = read_pathrecord(getpath(record_path2, "pospub").c_str(), position, line_num_read);
    //     ROS_INFO("cargothrough2");
    // }
    // else if (argv[1][0] == '3')
    // {
    //     line_num_read = read_pathrecord(getpath(record_path3, "pospub").c_str(), position, line_num_read);
    //     ROS_INFO("cargothrough3");
    // }
    // else if (argv[1][0] == '4')
    // {
    //     line_num_read = read_pathrecord(getpath(record_path4, "pospub").c_str(), position, line_num_read);
    //     ROS_INFO("cargothrough4");
    // }
    // else
    // {
    //     ROS_INFO("wrong input");
    // }
    // line_num_read = read_pathrecord(getpath(record_path_end, "pospub").c_str(), position, line_num_read);
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
    sub_cube = nh.subscribe("/cube", 1000, &cubemeasureCallback);
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
