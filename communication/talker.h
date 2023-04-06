#ifndef ROBOKIT_TALKER_H
#define ROBOKIT_TALKER_H
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <zmq.hpp>
#include "json.hpp"
#include "ConvertImage.h"
class Talker {
public:
	Talker();
	
	~Talker();
	void Init(const std::string &ip);
	void Pub(nlohmann::json &metadata);
	void Pub(cv::Mat &imput);
private:
	std::thread sub_thread;
	void Process();
	std::unique_ptr<ImagemConverter> converter_;
};


#endif //ROBOKIT_TALKER_H
