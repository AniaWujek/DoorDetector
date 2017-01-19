/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "LinesCornersFitting.hpp"
#include "Common/Logger.hpp"
#include <algorithm>
#include <boost/bind.hpp>

namespace Processors {
namespace LinesCornersFitting {

LinesCornersFitting::LinesCornersFitting(const std::string & name) :
		Base::Component(name) , 
		prop("prop", 0.1) {
	registerProperty(prop);

}

LinesCornersFitting::~LinesCornersFitting() {
}

void LinesCornersFitting::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_lines", &in_lines);
	registerStream("in_corners", &in_corners);
	registerStream("in_linesPairs", &in_linesPairs);
	registerStream("out_roiVec", &out_roiVec);
	registerStream("in_img", &in_img);
	registerStream("out_img", &out_img);
	// Register handlers
	registerHandler("LinesCornersFitting_processor", boost::bind(&LinesCornersFitting::LinesCornersFitting_processor, this));
	addDependency("LinesCornersFitting_processor", &in_lines);
	addDependency("LinesCornersFitting_processor", &in_linesPairs);
	addDependency("LinesCornersFitting_processor", &in_corners);

}

bool LinesCornersFitting::onInit() {

	return true;
}

bool LinesCornersFitting::onFinish() {
	return true;
}

bool LinesCornersFitting::onStop() {
	return true;
}

bool LinesCornersFitting::onStart() {
	return true;
}

float mi(float m, float start, float first_max, float last_max, float end) {
	if(m<=start || m>=end) {
		return 0.0;
	}
	if(m<first_max) {
		return (m-start)/(first_max-start);
	}
	if(m<last_max) {
		return 1.0;
	}
	if(m<end) {
		return (end-m)/(end-last_max);
	}
}


/* kat linii - funkcje przynaleznosci */
float mi_angle_H(float angle) {
	float a = mi(angle, 0.0, 0.0, 0.083*M_PI, 0.17*M_PI);
	float b = mi(angle,0.83*M_PI,0.92*M_PI,M_PI,M_PI);
	return std::max(a,b);
}
float mi_angle_UP(float angle) {
	float a = mi(angle,0.083*M_PI,0.17*M_PI,0.33*M_PI,0.42*M_PI);
	float b = mi(angle,0.58*M_PI,0.67*M_PI,0.83*M_PI,0.92*M_PI);
	return std::max(a,b);
}
float mi_angle_V(float angle) {
	return mi(angle,0.33*M_PI,0.42*M_PI,0.58*M_PI,0.67*M_PI);
}
/* ********************************/

/* rozmiar linii - funkcje przynaleznosci */
float mi_size_S(float size, float max) {
	return mi(size,0.0,0.0,0.25*max,0.38*max);
}
float mi_size_M(float size, float max) {
	return mi(size,0.25*max,0.38*max,0.63*max,0.75*max);
}
float mi_size_B(float size, float max) {
	return mi(size,0.63*max,0.75*max,max,max);
}
/* ********************************/

/* polozenie linii Y - funkcje przynaleznosci */
float mi_Ylocation_L(float loc, float max) {
	return mi(loc, 0.0, 0.0, 0.25*max, 0.38*max);
}
float mi_Ylocation_M(float loc, float max) {
	return mi(loc,0.25*max,0.38*max,0.63*max,0.75*max);
}
float mi_Ylocation_H(float loc, float max) {
	return mi(loc,0.63*max,0.75*max,max,max);
}
/* ********************************/

/* polozenie linii X - funkcje przynaleznosci */
float mi_Xlocation_L(float loc, float max) {
	return mi(loc, 0.0, 0.0, 0.25*max, 0.38*max);
}
float mi_Xlocation_M(float loc, float max) {
	return mi(loc,0.25*max,0.38*max,0.63*max,0.75*max);
}
float mi_Xlocation_R(float loc, float max) {
	return mi(loc,0.63*max,0.75*max,max,max);
}
/* ********************************/

float getAngle(cv::Vec4i line) {
	float x, y;
	x = line[2] - line[0];
	y = -line[3] + line[1];
	return atan2(y,x);
}

float getLength(cv::Point p0, cv::Point p1) {
    return sqrt((p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y));
}
float getLength(cv::Vec4i line) {
	return getLength(cv::Point(line[0],line[1]),cv::Point(line[2],line[3]));
}



void check_lines(std::vector<std::vector<std::vector<int> > > &v, std::vector<std::vector<std::vector<int> > > &h, std::vector<cv::Vec4i> &lines) {
	for(int i=0; i<lines.size(); ++i) {

		float a = getAngle(lines[i]);
		float s = getLength(lines[i]);

		std::vector<float> angles(3);
		angles[0] = mi_angle_H(a);
		angles[1] = mi_angle_UP(a);
		angles[2] = mi_angle_V(a);

		std::vector<float> sizes(3);
		sizes[0] = mi_size_S(s, 829.0);
		sizes[1] = mi_size_M(s, 829.0);
		sizes[2] = mi_size_B(s, 829.0);

		//vertical

		float Nver = 0.0;
		float Lver = 0.0;
		float Hver = 0.0;

		if(angles[0]>0.0) {
			if(sizes[0]>0.0) Nver = std::max(Nver,std::min(angles[0],sizes[0]));
			if(sizes[1]>0.0) Nver = std::max(Nver,std::min(angles[0],sizes[1]));
			if(sizes[2]>0.0) Nver = std::max(Nver,std::min(angles[0],sizes[2]));
		}
		if(angles[1]>0.0) {
			if(sizes[0]>0.0) Nver = std::max(Nver,std::min(angles[1],sizes[0]));
			if(sizes[1]>0.0) Lver = std::max(Lver,std::min(angles[1],sizes[1]));
			if(sizes[2]>0.0) Lver = std::max(Lver,std::min(angles[1],sizes[2]));
		} 
		if(angles[2]>0.0) {
			if(sizes[0]>0.0) Lver = std::max(Lver,std::min(angles[2],sizes[0]));
			if(sizes[1]>0.0) Hver = std::max(Hver,std::min(angles[2],sizes[1]));
			if(sizes[2]>0.0) Hver = std::max(Hver,std::min(angles[2],sizes[2]));
		}

		float Xpos = fabs(lines[i][0]+lines[i][2])/2.0;

		float Lside = mi_Xlocation_L(Xpos,622.0);
		float Mside = mi_Xlocation_M(Xpos,622.0);
		float Rside = mi_Xlocation_R(Xpos,622.0);

		if(Nver>=Lver && Nver>=Hver);//na pewno nie
		else {
			if(Lver>Hver) { //byc moze
				if(Lside>Mside && Lside>Rside) { //po lewej
					v[0][0].push_back(i);
				}
				else if(Mside>Rside) { //po srodku
					v[1][0].push_back(i);
				}
				else { //po prawej
					v[2][0].push_back(i);
				}
			}
			else { //na pewno
				if(Lside>Mside && Lside>Rside) { //po lewej
					v[0][1].push_back(i);
				}
				else if(Mside>Rside) { //po srodku
					v[1][1].push_back(i);
				}
				else { //po prawej
					v[2][1].push_back(i);
				}
			}
		}


		//horizontal

		float Nhor = 0.0;
		float Lhor = 0.0;
		float Hhor = 0.0;

		if(angles[0]>0.0) {
			if(sizes[0]>0.0) Hhor = std::max(Hhor,std::min(angles[0],sizes[0]));
			if(sizes[1]>0.0) Hhor = std::max(Hhor,std::min(angles[0],sizes[1]));
			if(sizes[2]>0.0) Lhor = std::max(Lhor,std::min(angles[0],sizes[2]));
		}
		if(angles[1]>0.0) {
			if(sizes[0]>0.0) Hhor = std::max(Hhor,std::min(angles[1],sizes[0]));
			if(sizes[1]>0.0) Hhor = std::max(Hhor,std::min(angles[1],sizes[1]));
			if(sizes[2]>0.0) Lhor = std::max(Lhor,std::min(angles[1],sizes[2]));
		} 
		if(angles[2]>0.0) {
			if(sizes[0]>0.0) Nhor = std::max(Nhor,std::min(angles[2],sizes[0]));
			if(sizes[1]>0.0) Nhor = std::max(Nhor,std::min(angles[2],sizes[1]));
			if(sizes[2]>0.0) Nhor = std::max(Nhor,std::min(angles[2],sizes[2]));
		}

		float Ypos = fabs(lines[i][1]+lines[i][3])/2.0;

		float Hhei = mi_Ylocation_L(Ypos,829.0);
		float Mhei = mi_Ylocation_M(Ypos,829.0);
		float Lhei = mi_Ylocation_H(Ypos,829.0);

		if(Nhor>=Lhor && Nhor>=Hhor);//na pewno nie
		else {
			if(Lhor>Hhor) { //byc moze
				if(Lhei>Mhei && Lhei>Hhei) { //na dole
					h[2][0].push_back(i);
				}
				else if(Mhei>Hhei) { //po srodku
					h[1][0].push_back(i);
				}
				else { //na gorze
					h[0][0].push_back(i);
				}
			}
			else { //na pewno
				if(Lhei>Mhei && Lhei>Hhei) { //na dole
					h[2][1].push_back(i);
				}
				else if(Mhei>Hhei) { //po srodku
					h[1][1].push_back(i);
				}
				else { //na dole
					h[0][1].push_back(i);
				}
			}
		}
	}
}

void LinesCornersFitting::LinesCornersFitting_processor() {

	std::vector<cv::Vec4i> lines = in_lines.read();
	std::vector<cv::Point> corners = in_corners.read();
	std::vector<std::pair<int,int> > lines_pairs = in_linesPairs.read();

	cv::Mat img = in_img.read().clone();

	/*
	       | probably | sure
	 ------------------------      
	left   |          |     
	middle |          |
	right  |          |
	*/
	std::vector<std::vector<std::vector<int> > > vertical;
	vertical.resize(3);
	vertical[0].resize(2);
	vertical[1].resize(2);
	vertical[2].resize(2);


	/*
	       | probably | sure
	 ------------------------      
	high   |          |     
	middle |          |
	low    |          |
	*/
	std::vector<std::vector<std::vector<int> > > horizontal;
	horizontal.resize(3);
	horizontal[0].resize(2);
	horizontal[1].resize(2);
	horizontal[2].resize(2);

	check_lines(vertical,horizontal,lines);

	/*for(int side=0; side<vertical.size(); ++side) {
		for(int prob1=0; prob1<vertical[side].size(); ++prob1) {
			for(int l=0; l<vertical[side][prob1].size(); ++l) {
				int idx = vertical[side][prob1][l];
				cv::Point p1 = cv::Point(lines[idx][0],lines[idx][1]);
				cv::Point p2 = cv::Point(lines[idx][2],lines[idx][3]);
				cv::Point p3 = cv::Point((lines[idx][0]+lines[idx][2])/2,(lines[idx][1]+lines[idx][3])/2);
				cv::line(img,p1,p2,cv::Scalar(0,255,0),3);
				if(side==0) {
					if(prob1==0) cv::putText(img,"VprobLeft",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
					else if(prob1==1) cv::putText(img,"sureV-Left",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
				}
				else if(side==1) {
					if(prob1==0) cv::putText(img,"VprobMid",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
					else if(prob1==1) cv::putText(img,"sureV-Mid",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
				}
				else if(side==2) {
					if(prob1==0) cv::putText(img,"VprobRight",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
					else if(prob1==1) cv::putText(img,"sureV-Right",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
				}
			}
		}
	}*/

	for(int hei=0; hei<horizontal.size(); ++hei) {
		for(int prob1=0; prob1<horizontal[hei].size(); ++prob1) {
			for(int l=0; l<horizontal[hei][prob1].size(); ++l) {
				int idx = horizontal[hei][prob1][l];
				cv::Point p1 = cv::Point(lines[idx][0],lines[idx][1]);
				cv::Point p2 = cv::Point(lines[idx][2],lines[idx][3]);
				cv::Point p3 = cv::Point((lines[idx][0]+lines[idx][2])/2,(lines[idx][1]+lines[idx][3])/2+10);
				cv::line(img,p1,p2,cv::Scalar(0,255,0),3);
				if(hei==0) {
					if(prob1==0) cv::putText(img,"probH-High",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
					else if(prob1==1) cv::putText(img,"sureH-High",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
				}
				else if(hei==1) {
					if(prob1==0) cv::putText(img,"probH-Mid",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
					else if(prob1==1) cv::putText(img,"sureH-Mid",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
				}
				else if(hei==2) {
					if(prob1==0) cv::putText(img,"probH-Low",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
					else if(prob1==1) cv::putText(img,"sureH-Low",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
				}
			}
		}
	}

	out_img.write(img);

	

	
}



} //: namespace LinesCornersFitting
} //: namespace Processors
