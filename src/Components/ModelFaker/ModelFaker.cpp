/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "ModelFaker.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ModelFaker {

ModelFaker::ModelFaker(const std::string & name) :
		Base::Component(name) ,
		width("width", 100), 
		height("height", 100) {
	registerProperty(width);
	registerProperty(height);

}

ModelFaker::~ModelFaker() {
}

void ModelFaker::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_img", &out_img);
	registerStream("in_rect", &in_rect);
	// Register handlers
	registerHandler("ModelFaker_processor", boost::bind(&ModelFaker::ModelFaker_processor, this));
	addDependency("ModelFaker_processor", &in_rect);

}

bool ModelFaker::onInit() {

	return true;
}

bool ModelFaker::onFinish() {
	return true;
}

bool ModelFaker::onStop() {
	return true;
}

bool ModelFaker::onStart() {
	return true;
}

void ModelFaker::ModelFaker_processor() {
	cv::Mat img = cv::Mat::zeros(height,width,CV_8U);
	std::vector<cv::Point2f> rect = in_rect.read();
	if(rect.size()==4) {
		cv::line(img, rect[0], rect[1], cv::Scalar(255,255,255), 3);
		cv::line(img, rect[1], rect[2], cv::Scalar(255,255,255), 3);
		cv::line(img, rect[2], rect[3], cv::Scalar(255,255,255), 3);
		cv::line(img, rect[3], rect[0], cv::Scalar(255,255,255), 3);
	}
	//std::cout<<"\n\n *** "<<rect.x<<" *** \n\n";
	//cv::rectangle(img, rect, cv::Scalar(255,255,255),3);
	out_img.write(img);
}



} //: namespace ModelFaker
} //: namespace Processors
