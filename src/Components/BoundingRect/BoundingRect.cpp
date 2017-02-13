/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "BoundingRect.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace BoundingRect {

BoundingRect::BoundingRect(const std::string & name) :
		Base::Component(name)  {

}

BoundingRect::~BoundingRect() {
}

void BoundingRect::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("out_rect", &out_rect);
	// Register handlers
	registerHandler("BoundingRect_processor", boost::bind(&BoundingRect::BoundingRect_processor, this));
	addDependency("BoundingRect_processor", &in_img);

}

bool BoundingRect::onInit() {

	return true;
}

bool BoundingRect::onFinish() {
	return true;
}

bool BoundingRect::onStop() {
	return true;
}

bool BoundingRect::onStart() {
	return true;
}

void BoundingRect::BoundingRect_processor() {
	cv::Mat img = in_img.read();

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;	
	cv::findContours(img,contours,hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );

	for(int i=1; i<contours.size(); ++i) {
		contours[0].insert(contours[0].end(), contours[i].begin(), contours[i].end());
	}

	cv::Rect rect = cv::boundingRect(contours[0]);

	std::vector<cv::Point2f> rectVec;
	rectVec.push_back(cv::Point2f(rect.x+rect.width,rect.y));
	rectVec.push_back(cv::Point2f(rect.x+rect.width,rect.y+rect.height));	
	rectVec.push_back(cv::Point2f(rect.x,rect.y+rect.height));	
	rectVec.push_back(cv::Point2f(rect.x,rect.y));

	out_rect.write(rectVec);


}



} //: namespace BoundingRect
} //: namespace Processors
