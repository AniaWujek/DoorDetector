/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "ModelSaver.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <iostream>
#include <fstream>

namespace Processors {
namespace ModelSaver {

ModelSaver::ModelSaver(const std::string & name) :
		Base::Component(name) , 
		path("path", std::string("/tmp/")) {
	registerProperty(path);

}

ModelSaver::~ModelSaver() {
}

void ModelSaver::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_features", &in_features);
	registerStream("in_descriptors", &in_descriptors);
	registerStream("in_boundingRect", &in_boundingRect);
	// Register handlers
	registerHandler("SaveFeatures", boost::bind(&ModelSaver::SaveFeatures, this));
	registerHandler("addModel", boost::bind(&ModelSaver::addModel, this));
	registerHandler("clearAll", boost::bind(&ModelSaver::clearAll, this));

}

bool ModelSaver::onInit() {

	return true;
}

bool ModelSaver::onFinish() {
	return true;
}

bool ModelSaver::onStop() {
	return true;
}

bool ModelSaver::onStart() {
	return true;
}

void ModelSaver::SaveFeatures() {
	if(allModels.size()>0 && allModels.size()==allDescriptors.size() && allModels.size()==allBoundingRect.size()) {
		for(int model=0; model<allModels.size(); ++model) {
			std::string file_path = std::string(path)+std::string("model_")+std::to_string(model)+std::string(".yml");
			cv::FileStorage fs(file_path, cv::FileStorage::WRITE);
			cv::write(fs,"keypoints", allModels[model].features);
			cv::write(fs,"descriptors", allDescriptors[model]);
			cv::write(fs,"boundingRect", allBoundingRect[model]);
			fs.release();
		}
		
	}
	else {
		CLOG(LERROR) << "No models to save!";
		return;
	}
}

void ModelSaver::addModel() {
	if(!in_features.empty()) {
		Types::Features model = in_features.read();
		allModels.push_back(model);
	}
	else {
		CLOG(LERROR) << "Empty feature buffer!";
	}
	if(!in_descriptors.empty()) {
		cv::Mat descriptor = in_descriptors.read();
		allDescriptors.push_back(descriptor);
	}
	else {
		CLOG(LERROR) << "Empty descriptors buffer!";
	}
	if(!in_boundingRect.empty()) {
		std::vector<cv::Point2f> rect = in_boundingRect.read();
		allBoundingRect.push_back(rect);
	}
	else {
		CLOG(LERROR) << "Empty rect buffer!";
	}
	
}

void ModelSaver::clearAll() {
	allModels.clear();
	allDescriptors.clear();
	allBoundingRect.clear();
}



} //: namespace ModelSaver
} //: namespace Processors
