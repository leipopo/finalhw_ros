#include <fstream>
#include <ros/package.h>
#include <string>

using namespace std;

string getpath(const char *path, const char *pkgname)
{
    string pkgpath = ros::package::getPath(pkgname);
    string filepath = pkgpath + path;
    return filepath;
}

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