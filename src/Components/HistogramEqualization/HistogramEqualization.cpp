/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "HistogramEqualization.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace HistogramEqualization {

HistogramEqualization::HistogramEqualization(const std::string & name) :
		Base::Component(name)  {

}

HistogramEqualization::~HistogramEqualization() {
}

void HistogramEqualization::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("onNewImage", boost::bind(&HistogramEqualization::onNewImage, this));
	addDependency("onNewImage", &in_img);

}

bool HistogramEqualization::onInit() {

	return true;
}

bool HistogramEqualization::onFinish() {
	return true;
}

bool HistogramEqualization::onStop() {
	return true;
}

bool HistogramEqualization::onStart() {
	return true;
}

void HistogramEqualization::onNewImage() {
	cv::Mat img = in_img.read().clone();
	cv::Mat output;
	cv::equalizeHist(img,output);

	out_img.write(output);
}



} //: namespace HistogramEqualization
} //: namespace Processors
