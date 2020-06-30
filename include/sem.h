//
// Created by moyaokai on 19-4-24.
//

#ifndef ORB_SLAM2_SEM_H
#define ORB_SLAM2_SEM_H

#include <vector>
#include <string>
#include <iostream>



namespace ORB_SLAM2
{

    class My_label{
    public:
        //对应的时间戳
        double timestamp = 0;
        //对应的帧的ID
        int frame = 0;
        //class label
        int label_class = 0;
        //position 中心点坐标、长和宽
        double position [4];
        double confidence = 0;
        My_label() {}
        ~My_label() {}
    };

    std::vector<std::string> split(std::string str, std::string pattern);
    std::vector<std::vector<My_label>>  readdata(std::string filename = "/home/moyaokai/server_for_ubuntu/semslam/usingtum/se_desk_bbx.txt");
    std::vector<double> compute_mean_and_stdev(std::vector<std::vector<double>> data);
    std::vector<double> compute_mean_and_stdev(std::vector<std::vector<int>> data);
    int quicksortonce(std::vector<double> &data, int low, int high);
    void quicksort(std::vector<double> &data, int low, int high);
    double evaluatemedian(std::vector<double> &data);
    std::vector<double> compute_median(std::vector<std::vector<double>> data);
    void record_detailed(std::vector<std::vector<double>> data);
}

#endif //ORB_SLAM2_SEM_H
