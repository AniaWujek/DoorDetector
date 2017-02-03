/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "DoorModel.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace DoorModel {

DoorModel::DoorModel(const std::string & name) :
		Base::Component(name) , 
		width("width", 0.8), 
		height("height", 2.0) {
	registerProperty(width);
	registerProperty(height);

	width.setCallback(boost::bind(&DoorModel::sizeCallback, this, _1, _2));
	height.setCallback(boost::bind(&DoorModel::sizeCallback, this, _1, _2));

}

DoorModel::~DoorModel() {
}

void DoorModel::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_points", &in_points);
	registerStream("out_doorModel", &out_doorModel);
	// Register handlers
	registerHandler("DoorModel_processor", boost::bind(&DoorModel::DoorModel_processor, this));
	addDependency("DoorModel_processor", &in_points);

}

bool DoorModel::onInit() {
	initModel();
	return true;
}

void DoorModel::initModel() {
	door_model = boost::shared_ptr<Types::Objects3D::Object3D>(new Types::Objects3D::Object3D());
	std::vector<cv::Point3f> modelPoints;

	modelPoints.push_back(cv::Point3f(width/2, height/2, 0));
	modelPoints.push_back(cv::Point3f(width/2, -height/2, 0));
	modelPoints.push_back(cv::Point3f(-width/2, -height/2, 0));
	modelPoints.push_back(cv::Point3f(-width/2, height/2, 0));

	door_model->setModelPoints(modelPoints);
}

void DoorModel::sizeCallback(float old_value, float new_value) {
	initModel();
}

bool DoorModel::onFinish() {
	return true;
}

bool DoorModel::onStop() {
	return true;
}

bool DoorModel::onStart() {
	return true;
}

void DoorModel::DoorModel_processor() {
	std::vector<cv::Point2f> points = in_points.read();
	if(points.size() == 4) {
		door_model->setImagePoints(points);
		out_doorModel.write(*door_model);
		//std::cout<<"\n *** \n"<<points<<"\n *** \n";
	}
}



} //: namespace DoorModel
} //: namespace Processors
