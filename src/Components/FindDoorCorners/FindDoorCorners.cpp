/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "FindDoorCorners.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace FindDoorCorners {

FindDoorCorners::FindDoorCorners(const std::string & name) :
		Base::Component(name),
		k_param("k_param",0.04),
		window_size("window_size", 10)  {

			registerProperty(k_param);
			registerProperty(window_size);

}

FindDoorCorners::~FindDoorCorners() {
}

void FindDoorCorners::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_img", &in_img);
	registerStream("in_lines", &in_lines);
	registerStream("out_points", &out_points);
	registerStream("out_linesPairs", &out_linesPairs);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("findDoorCorners_processor", boost::bind(&FindDoorCorners::findDoorCorners_processor, this));
	addDependency("findDoorCorners_processor", &in_img);
	addDependency("findDoorCorners_processor", &in_lines);

}

bool FindDoorCorners::onInit() {

	return true;
}

bool FindDoorCorners::onFinish() {
	return true;
}

bool FindDoorCorners::onStop() {
	return true;
}

bool FindDoorCorners::onStart() {
	return true;
}

bool getIntersectionPoint(cv::Vec4i line1, cv::Vec4i line2, cv::Point &p) {
	
	float A1 = line1[1]-line1[3];
	float B1 = line1[2]-line1[0];
	float C1 = -(line1[0]*line1[3] - line1[2]*line1[1]);
	float A2 = line2[1]-line2[3];
	float B2 = line2[2]-line2[0];
	float C2 = -(line2[0]*line2[3] - line2[2]*line2[1]);
	float D = A1*B2 - B1*A2;
	float Dx = C1*B2 - B1*C2;
	float Dy = A1*C2 - C1*A2;

	if(D>0.0 || D<0.0) {
		p.x = Dx/D;
		p.y = Dy/D;
	}
	else return false;

	return true;

}

float getLength(cv::Point p0, cv::Point p1) {
    return sqrt((p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y));
}
float getLength(cv::Vec4i line) {
	return getLength(cv::Point(line[0],line[1]),cv::Point(line[2],line[3]));
}

void FindDoorCorners::findDoorCorners_processor() {

	std::vector<cv::Vec4i> lines = in_lines.read();
	cv::Mat img = in_img.read().clone();
	cv::Mat mask;

	std::vector<std::pair<int,int> > lines_pairs;
	std::vector<cv::Point> corners;

	cv::Point intersection;

	for(int i=0; i<lines.size()-1; ++i) {
		for(int j=i+1; j<lines.size(); ++j) {
			if(getIntersectionPoint(lines[i], lines[j], intersection)) {
				if(intersection.x<0 || intersection.y<0 || intersection.x>=img.cols || intersection.y>=img.rows); //punkt przeciecia poza obrazkiem
				else {
						float t;
						if(getLength(lines[i])<getLength(lines[j])) t = getLength(lines[i]);
						else t = getLength(lines[j]);

						t /= 2.0;

						if(getLength(intersection,cv::Point(lines[i][0],lines[i][1]))<t || getLength(intersection,cv::Point(lines[i][2],lines[i][3]))<t
							|| getLength(intersection,cv::Point(lines[j][0],lines[j][1]))<t || getLength(intersection,cv::Point(lines[j][2],lines[j][3]))<t) {
							std::vector<cv::Point2f> feature_point;
							mask = cv::Mat::zeros(img.rows,img.cols,CV_8U);
							int ms = window_size;
							int x_corner = std::min(std::max(intersection.x-ms/2,0),img.cols-1);
							int y_corner = std::min(std::max(intersection.y-ms/2,0),img.rows-1);
							int x_side, y_side;
							if(x_corner+ms < img.cols) x_side = ms;
							else x_side = ms - (img.cols-x_corner);
							if(y_corner+ms < img.rows) y_side = ms;
							else y_side = ms - (img.rows-y_corner);

							mask(cv::Rect(x_corner, y_corner, x_side, y_side)) = 1;
							cv::goodFeaturesToTrack(img,feature_point,1,0.1,100,mask,3,1,k_param);

							

							if(feature_point.size() > 0) {
								corners.push_back(feature_point[0]);
								lines_pairs.push_back(std::make_pair(i,j));
							}
						}

						
				}
			}
		}
	}

	for(int i=0; i<corners.size(); ++i) {
		cv::circle(img, corners[i], 5, cv::Scalar(0,255,0),-1);
		int ms = 50;
		int x_corner = std::min(std::max(corners[i].x-ms/2,0),img.cols-1);
		int y_corner = std::min(std::max(corners[i].y-ms/2,0),img.rows-1);
		int x_side, y_side;
		if(x_corner+ms < img.cols) x_side = ms;
		else x_side = ms - (img.cols-x_corner);
		if(y_corner+ms < img.rows) y_side = ms;
		else y_side = ms - (img.rows-y_corner);
		cv::rectangle(img,cv::Point(x_corner,y_corner),cv::Point(x_corner+x_side,y_corner+y_side),cv::Scalar(0,255,0),3);
		
	}

	std::cout<<"\n*** "<<corners.size()<<" ***\n";

	out_img.write(img);

	out_points.write(corners);
	out_linesPairs.write(lines_pairs);
}



} //: namespace FindDoorCorners
} //: namespace Processors
