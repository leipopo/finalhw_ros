#include "numrec.hpp"

/*
 *功能：训练数字识别模型
 *输入：无
 */
int main(int argc, const char **argv)
{
    vector<string> path;
    vector<int> label;
    int nline = 1;
    string imgpath;
    ifstream cfgfile;
    cfgfile.open(getpath(traincfg_path, "numrec").c_str(), ios::in); // 打开训练索引文件
    while (!cfgfile.eof())
    {
        if (nline % 2 == 1)
        {
            cfgfile >> imgpath;
            path.push_back(imgpath);
        }
        else
        {
            int num;
            cfgfile >> num;
            label.push_back(num);
        }
        nline++;
    }
    cout << "total " << nline / 2 << endl;
    cfgfile.close();

    Mat labelmat = Mat(label.size(), 1, CV_32SC1);
    Mat datamat;
    int imgnum = path.size();
    cout << "imgnum " << imgnum << endl;
    for (int i = 0; i < imgnum; i++)
    {
        Mat gray_img = imread(path[i], IMREAD_GRAYSCALE); // 灰度图
        imshow("gray_img", gray_img);

        Mat blur_img;
        blur(gray_img, blur_img, Size(3, 3)); // 模糊
        // imshow("blur_img", blur_img);

        Mat thresh_img;
        threshold(blur_img, thresh_img, thresh_value, 255, THRESH_BINARY_INV); // 二值化
        // imshow("thresh_img", thresh_img);

        vector<vector<Point>> contours;                                                                 // 轮廓
        vector<Vec4i> hierarchy;                                                                        // 轮廓的结构信息
        findContours(thresh_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0)); // 查找轮廓

        vector<int> fea;

        Rect rect = boundingRect(contours[0]);
        Mat roi = thresh_img(rect);
        rectangle(thresh_img, rect, Scalar(255, 0, 255), 1, 8, 0);
        imshow("thresh_img_rect", thresh_img);
        Mat roi_resized;
        resize(roi, roi_resized, Size(imgx, imgy), 0, 0, INTER_AREA);
        imshow("roi_resized", roi_resized);
        waitKey(0);
        fea = roi_resized.reshape(1, 1);
        cout << "fea " << fea.size() << endl;
        if (i == 0)
            datamat = Mat::zeros(imgnum, fea.size(), CV_32FC1);
        for (int k = 0; k < fea.size(); k++)
        {
            datamat.at<float>(i, k) = fea[k];
            // cout<<fea.size()<<endl;
        }
        labelmat.at<int>(i, 0) = label[i];
        fea.clear();
    }

    if (argv[1][0] == 's')
    {
        deletefile(getpath(svm_path_result, "numrec").c_str());
        Ptr<ml::SVM> svm = ml::SVM::create();
        svm->setType(ml::SVM::C_SVC);
        svm->setKernel(ml::SVM::LINEAR);
        svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 100, 1e-6));
        svm->train(datamat, ml::ROW_SAMPLE, labelmat);
        svm->save(getpath(svm_path_result, "numrec").c_str());
        cout << "svm trained" << endl;
    }
    else if (argv[1][0] == 'k') // knn方法
    {

        deletefile(getpath(knn_path_result, "numrec").c_str());
        Ptr<ml::KNearest> knn = ml::KNearest::create();
        knn->setDefaultK(Kvalue);
        knn->setIsClassifier(true);
        knn->train(datamat, ml::ROW_SAMPLE, labelmat);
        knn->save(getpath(knn_path_result, "numrec").c_str());
        cout << "knn trained" << endl;
    }
    else
    {
        cout << "wrong input" << endl;
    }

    // waitKey(0);
    return 0;
}