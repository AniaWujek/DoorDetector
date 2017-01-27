/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "DoorElementsROI.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace DoorElementsROI {

DoorElementsROI::DoorElementsROI(const std::string & name) :
		Base::Component(name) , 
		ratio("ratio", 0.25),
		contour_n("contour_n",0),
		epsilon("epsilon",10) {
	registerProperty(ratio);
	registerProperty(contour_n);
	registerProperty(epsilon);

}

DoorElementsROI::~DoorElementsROI() {
}

void DoorElementsROI::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_door", &in_door);
	registerStream("in_img", &in_img);
	registerStream("out_elementsROI", &out_elementsROI);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("DoorElementsROI_processor", boost::bind(&DoorElementsROI::DoorElementsROI_processor, this));
	addDependency("DoorElementsROI_processor", &in_door);
	addDependency("DoorElementsROI_processor", &in_img);

}

bool DoorElementsROI::onInit() {

	return true;
}

bool DoorElementsROI::onFinish() {
	return true;
}

bool DoorElementsROI::onStop() {
	return true;
}

bool DoorElementsROI::onStart() {
	return true;
}

void drawPoly(cv::Mat &img, std::vector<cv::Point> points, cv::Scalar color) {
	cv::Point poly_points[1][points.size()];
	for(int i=0; i<points.size(); ++i) {
		poly_points[0][i] = points[i];
	}
	const cv::Point* ppt[1] = {poly_points[0]};
	int s = points.size();
	int npt[] = {s};
	cv::fillPoly(img,ppt,npt,1,color);
}

void DoorElementsROI::DoorElementsROI_processor() {

	cv::Mat img = in_img.read().clone();
	//cv::GaussianBlur(img,img,cv::Size(9,9),0,0);
	std::vector<cv::Point> door = in_door.read();

	
	



	cv::Mat edges, thresh_im, roi;
	cv::Rect br;
	float higherThreshold, lowerThreshold;

	cv::Mat mask = cv::Mat::zeros(img.rows,img.cols,CV_8U);
	drawPoly(mask,door,cv::Scalar(255,255,255));
	cv::bitwise_and(img,img,roi,mask);
	br = cv::boundingRect(door);
	higherThreshold = cv::threshold(roi(br), thresh_im, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
	lowerThreshold = ratio*higherThreshold;
	cv::Canny(roi,edges,lowerThreshold,higherThreshold);
	std::vector<vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::morphologyEx(edges,edges,cv::MORPH_GRADIENT,cv::Mat(), cv::Point(-1, -1),3);

	cv::findContours(edges,contours,hierarchy,CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
	cv::Mat drawing = cv::Mat::zeros( edges.size(), CV_8UC3 );
	for( int i = contours.size()-1; i>=0; i--) {
		std::vector<cv::Point> curve;
     	cv::approxPolyDP(contours[i],curve,epsilon,true);
     	contours[i] = curve;
		cv::Scalar color = cv::Scalar(0,0,255);
		cv::drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );
	}
	



	out_img.write(drawing);


}



} //: namespace DoorElementsROI
} //: namespace Processors
