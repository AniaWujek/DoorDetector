/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "QuadModel.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace QuadModel {

QuadModel::QuadModel(const std::string & name) :
		Base::Component(name) , 
		width("width", 0.8), 
		height("height", 2.0),
		type("type", 0) {
	registerProperty(width);
	registerProperty(height);
	registerProperty(type);

	width.setCallback(boost::bind(&QuadModel::sizeCallback, this, _1, _2));
	height.setCallback(boost::bind(&QuadModel::sizeCallback, this, _1, _2));

}

QuadModel::~QuadModel() {
}

void QuadModel::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_points", &in_points);
	registerStream("out_model", &out_model);
	// Register handlers
	registerHandler("QuadModel_processor", boost::bind(&QuadModel::QuadModel_processor, this));
	addDependency("QuadModel_processor", &in_points);

}

bool QuadModel::onInit() {
	initModel();
	return true;
}

void QuadModel::initModel() {
	model = boost::shared_ptr<Types::Objects3D::Object3D>(new Types::Objects3D::Object3D());
	std::vector<cv::Point3f> modelPoints;

	if(type==0) {
		modelPoints.push_back(cv::Point3f(width/2, height/2, 0));
		modelPoints.push_back(cv::Point3f(width/2, -height/2, 0));
		modelPoints.push_back(cv::Point3f(-width/2, -height/2, 0));
		modelPoints.push_back(cv::Point3f(-width/2, height/2, 0));
	}
	else if(type==1) {
		modelPoints.push_back(cv::Point3f(width, height, 0.0));
		modelPoints.push_back(cv::Point3f(width, 0.0, 0.0));
		modelPoints.push_back(cv::Point3f(0.0, 0.0, 0.0));
		modelPoints.push_back(cv::Point3f(0.0, height, 0));
	}

	model->setModelPoints(modelPoints);
}

void QuadModel::sizeCallback(float old_value, float new_value) {
	initModel();
}

bool QuadModel::onFinish() {
	return true;
}

bool QuadModel::onStop() {
	return true;
}

bool QuadModel::onStart() {
	return true;
}

void QuadModel::QuadModel_processor() {
	std::vector<cv::Point2f> points = in_points.read();
	if(points.size() == 4) {
		model->setImagePoints(points);
		out_model.write(*model);
		//std::cout<<"\n *** \n"<<points<<"\n *** \n";
	}
}



} //: namespace QuadModel
} //: namespace Processors
