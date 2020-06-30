
#ifndef ORB_SLAM2_YOLO_H
#define ORB_SLAM2_YOLO_H

#include <torch/torch.h>
#include <string>
#include <vector>
#include <map>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <sem.h>

using std::map;
using std::vector;
using std::string;

namespace ORB_SLAM2 {


struct Darknet : torch::nn::Module {

public:

	Darknet(const char *conf_file, torch::Device *device);

	map<string, string>* get_net_info();

	void load_weights(const char *weight_file);

	torch::Tensor forward(torch::Tensor x);

	/**
	 *  对预测数据进行筛选
	 */
	torch::Tensor write_results(torch::Tensor prediction, int num_classes, float confidence, float nms_conf = 0.4);

private:

	torch::Device *_device;

	vector<map<string, string>> blocks;

	torch::nn::Sequential features;

	vector<torch::nn::Sequential> module_list;

    // load YOLOv3
    void load_cfg(const char *cfg_file);

    void create_modules();

    int get_int_from_cfg(map<string, string> block, string key, int default_value);

    string get_string_from_cfg(map<string, string> block, string key, string default_value);
};


}// namespace ORB_SLAM


#endif //ORB_SLAM2_YOLO_H
