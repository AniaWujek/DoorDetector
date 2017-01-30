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
		epsilon("epsilon",10),
		horizontal_thresh("horizontal_thresh",10) {
	registerProperty(ratio);
	registerProperty(contour_n);
	registerProperty(epsilon);

	horizontal_thresh.addConstraint("0");
	horizontal_thresh.addConstraint("1000000");
	registerProperty(horizontal_thresh);



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

void drawPoly(cv::Mat &img, std::vector<cv::Point2f> points, cv::Scalar color) {
	cv::Point poly_points[1][points.size()];
	for(int i=0; i<points.size(); ++i) {
		poly_points[0][i] = cv::Point(points[i]);
	}
	const cv::Point* ppt[1] = {poly_points[0]};
	int s = points.size();
	int npt[] = {s};
	cv::fillPoly(img,ppt,npt,1,color);
}

void DoorElementsROI::DoorElementsROI_processor() {

	cv::Mat img = in_img.read().clone();
	//cv::GaussianBlur(img,img,cv::Size(9,9),0,0);
	std::vector<cv::Point2f> door = in_door.read();

	
	



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

	

	
	cv::Point2f p0, p1;
	cv::Point2f d0, d1, d2, d3;
	int old_count = cv::countNonZero(edges);
	int new_count = old_count;
	int margin = 10;
	cv::Mat d;
	int warunek = 0;
	do {
		d3 = cv::Point2f(door[3].x+margin,door[3].y+margin);
		d2 = cv::Point2f(door[2].x+margin,door[2].y-margin);
		d1 = cv::Point2f(door[1].x-margin,door[1].y-margin);
		d0 = cv::Point2f(door[0].x-margin,door[0].y+margin);
		//std::cout<<"\n*** points: "<<d0<<" "<<d1<<" ***\n\n";
		mask = cv::Mat::zeros(img.rows,img.cols,CV_8U);
		d = cv::Mat::zeros(img.rows,img.cols,CV_8U);
		std::vector<cv::Point2f> todraw = {d0,d1,d2,d3};
		drawPoly(mask,todraw,cv::Scalar(255,255,255));
		
		cv::bitwise_and(edges,edges,d,mask);
		old_count = new_count;
		new_count = cv::countNonZero(d);
		margin += 1;
		if(old_count - new_count < 10) warunek++;
		else warunek = 0;
		//std::cout<<"\n*** counts: "<<old_count<<" "<<new_count<<" ***\n\n";
	} while(warunek<2 && margin<30);

	//std::cout<<"\n*** margin: "<<margin<<" ***\n\n";


	cv::morphologyEx(d,d,cv::MORPH_GRADIENT,cv::Mat(), cv::Point(-1, -1),2);


	cv::Mat drawing = cv::Mat::zeros(img.rows,img.cols,CV_8UC3);
	cv::findContours(d,contours,hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );
	for( int i = contours.size()-1; i>=0; i--) {
		std::vector<cv::Point> curve;
     	cv::approxPolyDP(contours[i],curve,epsilon,true);
     	contours[i] = curve;
     	cv::convexHull(contours[i],curve);
     	contours[i] = curve;

     	cv::Moments mu = cv::moments(contours[i]);
     	cv::Point2f center_mass = cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00);
     	cv::circle(img,center_mass,5,cv::Scalar(255,255,255),-1);

		cv::drawContours( img, contours, i, cv::Scalar(0,0,255), 3, 8, hierarchy, 0, cv::Point() );
	}
	out_img.write(img);


}



} //: namespace DoorElementsROI
} //: namespace Processors
