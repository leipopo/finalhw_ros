#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/imgproc.hpp"
#include "opencv4/opencv2/ml.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdio.h>

using namespace cv;
using namespace std;

#define img_path "/home/lpga/finalhw_ros/src/numrec/img"
#define svm_path "/home/lpga/finalhw_ros/src/numrec/svm/svm.txt"
#define svm_path_result "/home/lpga/finalhw_ros/src/numrec/svm/svm_result.txt"

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

int main(int argc, const char **argv)
{

    vector<string> path;
    vector<int> label;
    int nline = 1;
    string imgpath;
    ifstream svmfile;
    svmfile.open(svm_path, ios::in);
    while (!svmfile.eof())
    {
        if (nline % 2 == 1)
        {
            svmfile >> imgpath;
            path.push_back(imgpath);
        }
        else
        {
            int num;
            svmfile >> num;
            label.push_back(num);
        }
        nline++;
    }
    cout << "total " << nline / 2 << endl;
    svmfile.close();

    Mat labelmat = Mat(label.size(), 1, CV_32SC1);
    Mat datamat;
    int imgnum = path.size();
    for (int i = 0; i < imgnum; i++)
    {
        Mat img = imread(path[i], 0);
        Mat thresh_img;
        threshold(img, thresh_img, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(thresh_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0)); //
        Mat mask = Mat::zeros(thresh_img.size(), CV_8UC1);
        drawContours(mask, contours, -1, Scalar(255), -1); // 绘制所有轮廓
        imshow("mask", mask);
        vector<int> fea;
        for (int j = 0; j < contours.size(); j++)
        {
            Rect rect = boundingRect(contours[j]);
            Mat roi = img(rect);
            Mat roi_resized;
            resize(roi, roi_resized, Size(180, 210), 0, 0, INTER_AREA);
            int g_dConArea = contourArea(contours[j]);
            if (g_dConArea > 0)
            {
                cout << "用轮廓面积计算函数计算出来的第" << j << "个轮廓的面积为： " << g_dConArea << endl;
                int value1 = 0;
                for (int k = 0; k < roi_resized.rows; k++)
                {
                    for (int l = 0; l < 6; l++)
                    {
                        value1 += roi_resized.at<uchar>(k, l);
                    }
                }
                cout << "像素值之和 " << value1 << endl;
                fea.push_back(value1);
            }
        }
        cout << "feasize " << fea.size() << endl;
        if (i == 0)
            datamat = Mat::zeros(imgnum, fea.size(), CV_32FC1);
        for (int j = 0; j < fea.size(); j++)
        {
            datamat.at<float>(i, j) = fea[j];
        }
        labelmat.at<int>(i, 0) = label[i];
        cout << "label " << label[i] << endl;
        fea.clear();
    }
    
    
    Ptr<ml::SVM> svm = ml::SVM::create();
    svm->setType(ml::SVM::C_SVC);
    svm->setKernel(ml::SVM::LINEAR);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));
    svm->train(datamat, ml::ROW_SAMPLE, labelmat);
    svm->save(svm_path_result);
    cout << "svm trained" << endl;
    return 0;
}