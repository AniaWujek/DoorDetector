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
		horizontal_thresh("horizontal_thresh",10),
		additional_check("additional_check", 0) {
	registerProperty(ratio);
	registerProperty(contour_n);
	registerProperty(epsilon);

	horizontal_thresh.addConstraint("0");
	horizontal_thresh.addConstraint("1000000");
	registerProperty(horizontal_thresh);
	registerProperty(additional_check);



}

DoorElementsROI::~DoorElementsROI() {
}

void DoorElementsROI::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_door", &in_door);
	registerStream("in_img", &in_img);
	registerStream("out_elementsROI", &out_elementsROI);
	registerStream("out_img", &out_img);
	registerStream("out_elementCenters", &out_elementCenters);
	registerStream("out_elementCentersVec", &out_elementCentersVec);
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

float getTriangleArea(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2) {
	return fabs(((p0.x-p2.x)*(p1.y-p0.y)-(p0.x-p1.x)*(p2.y-p0.y))/2.0);
}

float getLength(cv::Point p0, cv::Point p1) {
    return sqrt((p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y));
}

void DoorElementsROI::DoorElementsROI_processor() {

	cv::Mat img = in_img.read().clone();
	//cv::GaussianBlur(img,img,cv::Size(9,9),0,0);
	std::vector<cv::Point2f> door = in_door.read();

	if(door.size()==4) {

	
	



		cv::Mat edges, thresh_im, roi;
		cv::Rect br;
		float higherThreshold, lowerThreshold;

		cv::Mat mask = cv::Mat::zeros(img.rows,img.cols,CV_8U);
		drawPoly(mask,door,cv::Scalar(255,255,255));
		cv::bitwise_and(img,img,roi,mask);
		cv::Mat thresh;
		cv::adaptiveThreshold(roi,thresh, 255,cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 9, 5);

		br = cv::boundingRect(door);
		higherThreshold = cv::threshold(thresh(br), thresh_im, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
		lowerThreshold = ratio*higherThreshold;
		cv::Canny(thresh,edges,lowerThreshold,higherThreshold);
		std::vector<std::vector<cv::Point> > contours;
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

		std::vector<cv::Point2f> element_centers;

		cv::Mat drawing = cv::Mat::zeros(img.rows,img.cols,CV_8UC3);
		cv::findContours(d,contours,hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0) );

		float door_area = getTriangleArea(door[0],door[1],door[2]) + getTriangleArea(door[0],door[2],door[3]);
		float door_height = (door[1].y-door[0].y + door[2].y-door[3].y)/2.0;
		float door_width = (door[0].x-door[3].x + door[1].x-door[2].x)/2.0;
		float left = (door[3].x + door[2].x)/2.0;
		float right = (door[0].x + door[1].x)/2.0;
		float top = (door[3].y + door[0].y)/2.0;
		float bottom = (door[2].y + door[1].y)/2.0;
		for( int i = contours.size()-1; i>=0; i--) {
			std::vector<cv::Point> curve;
	     	cv::approxPolyDP(contours[i],curve,epsilon,true);
	     	contours[i] = curve;
	     	cv::convexHull(contours[i],curve);
	     	contours[i] = curve;

	     	cv::Moments mu = cv::moments(contours[i]);
	     	cv::Point2f center_mass = cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00);

	     	float contour_area = cv::contourArea(contours[i]);

	     	bool good_center = true;
	     	if(additional_check) {
	     		good_center = false;
	     		good_center = fabs(center_mass.y-top)/door_height>0.1 && fabs(center_mass.y-bottom)/door_height>0.4
		     	&& (fabs(center_mass.x-left)/door_width<0.2 || fabs(center_mass.x-right)/door_width<0.2 || (fabs(center_mass.x-left)/door_width>0.3
		     		&& fabs(center_mass.x-right)/door_width>0.3) && contour_area/door_area < 0.1);
	     	}
	     	

	     	if(good_center && center_mass.x>=0 && center_mass.x<img.cols && center_mass.y>=0 && center_mass.y<img.rows ) {
	     		float min_dist = door_width;
	     		int min_contour = -1;
	     		for(int i=0; i<element_centers.size(); ++i) {
	     			float dist = getLength(element_centers[i],center_mass);
	     			if(dist<min_dist) {
	     				min_dist = dist;
	     				min_contour = i;
	     			}
	     		}
	     		if(min_dist>0.2 * door_width) {
		     		cv::circle(img,center_mass,10,cv::Scalar(0,0,0),-1);

			     	element_centers.push_back(center_mass);

					cv::drawContours( img, contours, i, cv::Scalar(0,0,0), 5, 8, hierarchy, 0, cv::Point() );
				}
	     	}

	     	
		}

		std::vector<std::vector<float> > elements_centersVec;
		for(int i=0; i<element_centers.size(); ++i) {
			std::vector<float> p;
			p.push_back(element_centers[i].x);
			p.push_back(element_centers[i].y);
			elements_centersVec.push_back(p);
		}
		out_img.write(img);
		out_elementCenters.write(element_centers);
		out_elementCentersVec.write(elements_centersVec);
	}


}



} //: namespace DoorElementsROI
} //: namespace Processors
