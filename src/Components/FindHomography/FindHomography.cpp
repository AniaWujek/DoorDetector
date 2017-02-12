/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "FindHomography.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace FindHomography {

FindHomography::FindHomography(const std::string & name) :
		Base::Component(name)  {

}

FindHomography::~FindHomography() {
}

void FindHomography::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_modelPoints", &in_modelPoints);
	registerStream("in_objectPoints", &in_objectPoints);
	registerStream("out_homography", &out_homography);
	// Register handlers
	registerHandler("FindHomography_processor", boost::bind(&FindHomography::FindHomography_processor, this));
	addDependency("FindHomography_processor", &in_modelPoints);
	addDependency("FindHomography_processor", &in_objectPoints);

}

bool FindHomography::onInit() {

	return true;
}

bool FindHomography::onFinish() {
	return true;
}

bool FindHomography::onStop() {
	return true;
}

bool FindHomography::onStart() {
	return true;
}

void FindHomography::FindHomography_processor() {
}



} //: namespace FindHomography
} //: namespace Processors
