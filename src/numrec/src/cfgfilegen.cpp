#include "numrec.hpp"

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

int main(int argc, char **argv)
{
    int numofnums = atoi(argv[1]);
    int numofeachimg = atoi(argv[2]);

    cout << "total " << numofnums << endl;
    cout << "each " << numofeachimg << endl;

    string Img_Path = img_path;
    deletefile(traincfg_path);
    for (int i = 1; i < numofnums+1; i++)
    {
        for (int j = 0; j < numofeachimg; j++)
        {
            string imgpath = Img_Path + "/num" + to_string(i) + "-" + to_string(j) + ".jpg" + "\n";
            writefile(traincfg_path, imgpath);
            writefile(traincfg_path, to_string(i) + "\n");
        }
    }

    return 0;
}