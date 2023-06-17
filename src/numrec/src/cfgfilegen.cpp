#include "numrec.hpp"

int main(int argc, char **argv)
{
    int numofnums = atoi(argv[1]);
    int numofeachimg = atoi(argv[2]);

    cout << "total " << numofnums << endl;
    cout << "each " << numofeachimg << endl;

    string Img_Path = getpath(img_path, "numrec").c_str();
    deletefile(getpath(traincfg_path, "numrec").c_str());
    for (int i = 1; i < numofnums + 1; i++)
    {
        for (int j = 0; j < numofeachimg; j++)
        {
            string imgpath = Img_Path + "/num" + to_string(i) + "-" + to_string(j) + ".jpg" + "\n";
            writefile(getpath(traincfg_path, "numrec").c_str(), imgpath);
            writefile(getpath(traincfg_path, "numrec").c_str(), to_string(i) + "\n");
        }
    }

    return 0;
}