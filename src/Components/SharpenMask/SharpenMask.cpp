/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "SharpenMask.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace SharpenMask {

SharpenMask::SharpenMask(const std::string & name) :
		Base::Component(name) , 
		kernel_width("kernel_width", 1, "range"), 
		kernel_height("kernel_height", 1, "range"), 
		sigmax("sigmax", 0.0), 
		sigmay("sigmay", 0.0), 
		threshold("threshold", 5), 
		amount("amount", 1) {

	kernel_width.addConstraint("0");
	kernel_width.addConstraint("10");
	kernel_height.addConstraint("0");
	kernel_height.addConstraint("10");

	registerProperty(kernel_width);
	registerProperty(kernel_height);
	registerProperty(sigmax);
	registerProperty(sigmay);
	registerProperty(threshold);
	registerProperty(amount);

}

SharpenMask::~SharpenMask() {
}

void SharpenMask::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("onNewImage", boost::bind(&SharpenMask::onNewImage, this));
	addDependency("onNewImage", &in_img);

}

bool SharpenMask::onInit() {

	return true;
}

bool SharpenMask::onFinish() {
	return true;
}

bool SharpenMask::onStop() {
	return true;
}

bool SharpenMask::onStart() {
	return true;
}

void SharpenMask::onNewImage() {

	cv::Mat img = in_img.read().clone();
	cv::Mat blurred;
	cv::GaussianBlur(img,blurred,cv::Size(kernel_width, kernel_height), sigmax, sigmay);
	/*cv::Mat mask = abs(img - blurred) < threshold;
	cv::Mat sharpened = img*(1+amount) + blurred*(-amount);
	img.copyTo(sharpened,mask);
	out_img.write(img);*/

	cv::Mat lap;
	int d = CV_16S;
	cv::cvtColor( blurred, blurred, CV_BGR2GRAY );
	cv::Laplacian(blurred,lap,d,threshold,amount,0,cv::BORDER_DEFAULT);
	cv::Mat output;
	cv::convertScaleAbs(lap,output);
	out_img.write(output);
}



} //: namespace SharpenMask
} //: namespace Processors
