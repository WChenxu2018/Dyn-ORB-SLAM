
#include "sem.h"
#include <fstream>
#include <sstream>
#include <numeric>
#include <algorithm>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using std::string;
using std::vector;
using cv::Mat;
using cv::String;

namespace ORB_SLAM2{

    std::vector<std::string> split(std::string str, std::string pattern){
        std::string::size_type pos;
        std::vector<std::string> result;
        str += pattern;
        int size = str.size();

        for(int i = 0; i < size; i++){
            pos = str.find(pattern, i);
            if(pos<size){
                std::string s=str.substr(i, pos-i);
                result.push_back(s);
                i = pos + pattern.size() - 1;

            }
        }
        return result;
    }

    std::vector<std::vector<My_label>> readdata(std::string filename) {
        //总的label信息集合
        std::vector<std::vector<My_label>> myLabel_set;

        std::vector<My_label> myLabel;

        std::ifstream myFIle;

        std::ifstream myData;

        myFIle.open(filename, std::ios_base::in);

        if (myFIle.is_open()) {

            string files;
            int frame = 0;
            while (myFIle.good()) {
                //逐行读取文件中的文件名
                getline(myFIle, files);
                if (files != "") {
                    myData.open(files, std::ios_base::in);
                    if (myData.is_open()) {

                        while (myData.good()) {


                            string lines = "org";
                            My_label single_label;
                            string data_line[5];

                            getline(myData, lines);

                            if (lines == "")
                                continue;

                            std::istringstream is(lines);
                            is >> data_line[0] >> data_line[1] >> data_line[2] >> data_line[3] >> data_line[4];
                            single_label.label_class = atoi(data_line[0].c_str());
                            for (int j = 1; j < 5; j++) {
                                single_label.position[j - 1] = atof(data_line[j].c_str());
                            }
                            single_label.frame = frame;
                            myLabel.push_back(single_label);
                        }
                        myData.close();
                    }
                    myLabel_set.push_back(myLabel);
                    frame++;
                    myLabel.clear();
                }
            }
            myFIle.close();
        } else
            std::cout << "open() failed" << std::endl;

        return myLabel_set;

    }

    std::vector<double> compute_mean_and_stdev(std::vector<std::vector<double>> data){
        std::vector<double> result;
        for(int i = 0; i < 3; i++){
            std::vector<double> temp;
            for(int j = 0; j < data.size(); j++){
                if(data[j][i] >= 0)
                temp.push_back(data[j][i]);
            }
            double sum = std::accumulate(std::begin(temp), std::end(temp), 0.0);
            double mean = sum / temp.size();

            double accum = 0.0;
            std::for_each(std::begin(temp), std::end(temp), [&](const double d){
                accum += (d-mean)*(d-mean);
            });
            double stdev = sqrt(accum/(temp.size()));
            result.push_back(mean);
            result.push_back(stdev);
        }
        return result;
    }

    std::vector<double> compute_mean_and_stdev(std::vector<std::vector<int>> data){

        std::vector<double> result;
        for(int i = 0; i < 3; i++){
            std::vector<int> temp;
            for(int j = 0; j < data.size(); j++){
                temp.push_back(data[j][i]);
            }
            unsigned long sum = std::accumulate(std::begin(temp), std::end(temp), 0.0);

            double mean = double(sum) / temp.size();

            double accum = 0.0;
            std::for_each(std::begin(temp), std::end(temp), [&](const double d){
                accum += (d-mean)*(d-mean);
            });
            double stdev = sqrt(accum/(temp.size()));
            result.push_back(mean);
            result.push_back(stdev);
        }
        return result;
    }

    int quicksortonce(std::vector<double> &data, int low, int high){
        double temp = data[low];
        int i = low;
        int j = high;

        while(i < j){
            while (data[j] >= temp && i < j){
                j--;
            }

            data[i] = data[j];

            while (data[i] <= temp && i < j){
                i++;
            }

            data[j] = data[i];
        }

        data[i] = temp;

        return i;
    }

    void quicksort(std::vector<double> &data, int low, int high){
        if(low >= high){
            return;
        }

        int index = quicksortonce(data, low, high);

        quicksort(data, low, index - 1);
        quicksort(data, index + 1, high);
    }

    double evaluatemedian(std::vector<double> &data){
        int n = data.size();
        quicksort(data, 0, n-1);

        if(n%2 != 0){
            return data[(n+1)/2];
        }
        else{
            return (data[n/2] + data[n/2 - 1])/2;
        }
    }

    std::vector<double> compute_median(std::vector<std::vector<double>> data){
        std::vector<double> result;
        for(int i = 0; i < data[0].size(); i++){
            std::vector<double> temp;
            for(int j = 0; j < data.size(); j++){
                if(data[j][i] >= 0)
                temp.push_back(data[j][i]);
            }
            //
            double sum = std::accumulate(std::begin(temp), std::end(temp), 0.0);
            std::cout << i << " -> " << sum << std::endl;
            double median = evaluatemedian(temp);
            result.push_back(median);
        }

        return result;
    }

    void record_detailed(std::vector<std::vector<double>> data){
        string filename = "record_detail.txt";
        std::ofstream record;
        record.open(filename);
        if(record.is_open()){
            for(int j = 0; j < data.size(); j++){
                if(data[j][1] >= 0)
                    record << data[j][1] << std::endl;
                else record << 0 << std::endl;
            }
        }
        record.close();
    }



}