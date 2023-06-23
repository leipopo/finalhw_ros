#ifndef POSPUB_HPP
#define POSPUB_HPP
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>

using namespace std;

#define record_path_begin "/pathrec/pathrecordbegin.txt"
#define record_path1 "/pathrec/pathrecord1.txt"
#define record_path2 "/pathrec/pathrecord2.txt"
#define record_path3 "/pathrec/pathrecord3.txt"
#define record_path4 "/pathrec/pathrecord4.txt"
#define record_path_end "/pathrec/pathrecordend.txt"

string getpath(const char *path, const char *pkgname);
void writefile(const char *path, string data);
string readfile(const char *path);
void deletefile(const char *path);
void string2char(string str, char *ch);


#endif
