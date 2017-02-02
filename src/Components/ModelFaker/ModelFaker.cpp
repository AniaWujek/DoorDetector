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
		positionX("positionX", 50),
		positionY("positionY", 50), 
		width("width", 100), 
		height("height", 100), 
		radius("radius", 10) {
	registerProperty(positionX);
	registerProperty(width);
	registerProperty(height);
	registerProperty(radius);
	registerProperty(positionY);

}

ModelFaker::~ModelFaker() {
}

void ModelFaker::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("ModelFaker_processor", boost::bind(&ModelFaker::ModelFaker_processor, this));
	addDependency("ModelFaker_processor", NULL);

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
	cv::circle(img, cv::Point(positionX,positionY), radius, 255, -1);
	out_img.write(img);
}



} //: namespace ModelFaker
} //: namespace Processors
