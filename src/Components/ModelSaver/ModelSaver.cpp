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
	if(allModels.size()>0 && allDescriptors.size()>0 && allModels.size()==allDescriptors.size()) {
		for(int model=0; model<allModels.size(); ++model) {
			std::string file_path = std::string(path)+std::string("model_")+std::to_string(model)+std::string(".yml");
			cv::FileStorage fs(file_path, cv::FileStorage::WRITE);
			cv::write(fs,"keypoints", allModels[model].features);
			cv::write(fs,"descriptors", allDescriptors[model]);
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
	if(!in_descriptors.empty()) {
		cv::Mat descriptor = in_descriptors.read();
		allDescriptors.push_back(descriptor);
	}
	else {
		CLOG(LERROR) << "Empty buffers!";
	}
}

void ModelSaver::clearAll() {
	allModels.clear();
	allDescriptors.clear();
}



} //: namespace ModelSaver
} //: namespace Processors
