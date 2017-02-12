/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "ModelLoader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ModelLoader {

ModelLoader::ModelLoader(const std::string & name) :
		Base::Component(name) , 
		directory("directory", std::string("/tmp/")), 
		pattern("pattern", std::string(".*\\.(jpg|png|bmp|yaml|yml)")) {
	registerProperty(directory);
	registerProperty(pattern);

}

ModelLoader::~ModelLoader() {
}

void ModelLoader::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_features", &out_features);
	registerStream("out_descriptors", &out_descriptors);
	registerStream("out_boundingRect", &out_boundingRect);
	registerStream("out_names", &out_names);
	// Register handlers
	registerHandler("LoadModels", boost::bind(&ModelLoader::LoadModels, this));
	addDependency("LoadModels", NULL);
	registerHandler("ReloadModels", boost::bind(&ModelLoader::ReloadModels, this));

}

bool ModelLoader::findFiles() {
	files.clear();

	files = Utils::searchFiles(directory, pattern);

	std::sort(files.begin(), files.end());

	CLOG(LINFO) << "Sequence loaded.";
	BOOST_FOREACH(std::string fname, files)
		CLOG(LINFO) << fname;

	return !files.empty();
}

bool ModelLoader::onInit() {
	
	ReloadModels();
	return true;
}

bool ModelLoader::onFinish() {
	return true;
}

bool ModelLoader::onStop() {
	return true;
}

bool ModelLoader::onStart() {
	return true;
}

void ModelLoader::ReloadModels() {
	features.clear();
	descriptors.clear();
	boundingRect.clear();
	
	if(findFiles()) {
		for(int file=0; file<files.size(); ++file) {
			cv::FileStorage fs(files[file],cv::FileStorage::READ);

			cv::FileNode fn = fs["keypoints"];			
			std::vector<cv::KeyPoint> kp;
			cv::read(fn, kp);
			Types::Features f(kp);
			features.push_back(f);

			fn = fs["descriptors"];
			cv::Mat d;
			cv::read(fn, d);
			descriptors.push_back(d);

			fn = fs["boundingRect"];
			std::vector<cv::Point2f> b;
			cv::read(fn, b);
			boundingRect.push_back(b);
		}
		
	}
	else CLOG(LERROR) << "No files found!";
}

void ModelLoader::LoadModels() {
	CLOG(LNOTICE) << files.size() << " files, " << descriptors.size() << " descriptors, " << features.size() << " features";
	out_descriptors.write(descriptors);
	out_features.write(features);
	out_boundingRect.write(boundingRect);
	out_names.write(files);

}


} //: namespace ModelLoader
} //: namespace Processors
