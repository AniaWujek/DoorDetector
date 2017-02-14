/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "LinesCornersFitting2.hpp"
#include "Common/Logger.hpp"
#include <algorithm>
#include <boost/bind.hpp>
#include <iomanip>

namespace Processors {
namespace LinesCornersFitting2 {

LinesCornersFitting2::LinesCornersFitting2(const std::string & name) :
		Base::Component(name) , 
		prop("prop", 0),
		quality("quality",3),
		dev_choice("dev_choice", 0),
		hsv_channel("hsv_channel", 0),
		width("width", 0),
		height("height", 0),
		size_ratio_min("size_ratio_min", 1.5),
		size_ratio_max("size_ratio_max", 3.0) {
	registerProperty(prop);
	registerProperty(quality);
	registerProperty(dev_choice);
	registerProperty(width);
	registerProperty(height);
	registerProperty(size_ratio_min);
	registerProperty(size_ratio_max);

	hsv_channel.addConstraint("0");
	hsv_channel.addConstraint("2");
	registerProperty(hsv_channel);

}

LinesCornersFitting2::~LinesCornersFitting2() {
}

void LinesCornersFitting2::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_lines", &in_lines);
	registerStream("in_corners", &in_corners);
	registerStream("in_linesPairs", &in_linesPairs);
	registerStream("out_door", &out_door);
	registerStream("in_img", &in_img);
	registerStream("out_img", &out_img);
	registerStream("out_doorVec", &out_doorVec);
	// Register handlers
	registerHandler("LinesCornersFitting2_processor", boost::bind(&LinesCornersFitting2::LinesCornersFitting2_processor, this));
	addDependency("LinesCornersFitting2_processor", &in_lines);
	addDependency("LinesCornersFitting2_processor", &in_linesPairs);
	addDependency("LinesCornersFitting2_processor", &in_corners);
	addDependency("LinesCornersFitting2_processor", &in_img);

}

bool LinesCornersFitting2::onInit() {

	return true;
}

bool LinesCornersFitting2::onFinish() {
	return true;
}

bool LinesCornersFitting2::onStop() {
	return true;
}

bool LinesCornersFitting2::onStart() {
	return true;
}

float mi(float m, float start, float first_max, float last_max, float end) {
	if(m<start || m>end) {
		return 0.0;
	}
	if(m<first_max) {
		if(first_max==start) return 1.0;
		return (m-start)/(first_max-start);
	}
	if(m<last_max) {
		return 1.0;
	}
	if(m<end) {
		if(end==last_max) return 1.0;
		return (end-m)/(end-last_max);
	}
}


/* kat linii - funkcje przynaleznosci */
float mi_angle_H(float angle) {
	float a = mi(angle, 0.0, 0.0, 0.15*M_PI, 0.25*M_PI);
	float b = mi(angle,0.75*M_PI,0.85*M_PI,M_PI,M_PI);
	return std::max(a,b);
}
float mi_angle_UP(float angle) {
	float a = mi(angle,0.15*M_PI,0.25*M_PI,0.25*M_PI,0.35*M_PI);
	float b = mi(angle,0.65*M_PI,0.75*M_PI,0.75*M_PI,0.85*M_PI);
	return std::max(a,b);
}
float mi_angle_V(float angle) {
	return mi(angle,0.25*M_PI,0.35*M_PI,0.65*M_PI,0.75*M_PI);
}
/* ********************************/

/* rozmiar linii - funkcje przynaleznosci */
float mi_size_VS(float size, float max) {
	return mi(size,0.0,0.0,0.10*max,0.2*max);
}
float mi_size_S(float size, float max) {
	return mi(size,0.1*max,0.2*max,0.3*max,0.45*max);
}
float mi_size_M(float size, float max) {
	return mi(size,0.30*max,0.45*max,0.70*max,0.80*max);
}
float mi_size_B(float size, float max) {
	return mi(size,0.70*max,0.80*max,max,max);
}
/* ********************************/

/* polozenie linii Y - funkcje przynaleznosci */
float mi_Ylocation_L(float loc, float max) {
	return mi(loc, 0.0, 0.0, 0.25*max, 0.4*max);
}
float mi_Ylocation_M(float loc, float max) {
	return mi(loc,0.25*max,0.4*max,0.6*max,0.75*max);
}
float mi_Ylocation_H(float loc, float max) {
	return mi(loc,0.6*max,0.75*max,max,max);
}
/* ********************************/

/* polozenie linii X - funkcje przynaleznosci */
float mi_Xlocation_L(float loc, float max) {
	return mi(loc, 0.0, 0.0, 0.25*max, 0.5*max);
}
float mi_Xlocation_M(float loc, float max) {
	return mi(loc,0.25*max,0.5*max,0.5*max,0.75*max);
}
float mi_Xlocation_R(float loc, float max) {
	return mi(loc,0.5*max,0.75*max,max,max);
}
/* ********************************/

float getAngle(cv::Vec4i line) {
	float x, y;
	x = line[2] - line[0];
	y = -line[3] + line[1];
	return atan2(y,x);
}

float getLength(cv::Point2f p0, cv::Point2f p1) {
    return sqrt((p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y));
}
float getLength(cv::Vec4i line) {
	return getLength(cv::Point2f(line[0],line[1]),cv::Point2f(line[2],line[3]));
}



void LinesCornersFitting2::check_lines(std::vector<std::vector<std::vector<int> > > &v, std::vector<std::vector<std::vector<int> > > &h, std::vector<cv::Vec4i> &lines) {
	for(int i=0; i<lines.size(); ++i) {

		float a = getAngle(lines[i]);
		float s = getLength(lines[i]);

		std::vector<float> angles(3);
		angles[0] = mi_angle_H(a);
		angles[1] = mi_angle_UP(a);
		angles[2] = mi_angle_V(a);

		std::vector<float> sizes(4);
		float diag = sqrt(float(width*width)+float(height*height));
		//float diag = height;
		sizes[0] = mi_size_VS(s, diag);
		sizes[1] = mi_size_S(s, diag);
		sizes[2] = mi_size_M(s, diag);
		sizes[3] = mi_size_B(s, diag);

		//vertical

		float Nver = 0.0;
		float Lver = 0.0;
		float Hver = 0.0;

		if(angles[0]>0.0) {
			if(sizes[0]>0.0) Nver = std::max(Nver,std::min(angles[0],sizes[0]));
			if(sizes[1]>0.0) Nver = std::max(Nver,std::min(angles[0],sizes[1]));
			if(sizes[2]>0.0) Nver = std::max(Nver,std::min(angles[0],sizes[2]));
			if(sizes[3]>0.0) Nver = std::max(Nver,std::min(angles[0],sizes[3]));
		}
		if(angles[1]>0.0) {
			if(sizes[0]>0.0) Nver = std::max(Nver,std::min(angles[1],sizes[0]));
			if(sizes[1]>0.0) Nver = std::max(Nver,std::min(angles[1],sizes[1]));
			if(sizes[2]>0.0) Lver = std::max(Lver,std::min(angles[1],sizes[2]));
			if(sizes[3]>0.0) Lver = std::max(Lver,std::min(angles[1],sizes[3]));
		} 
		if(angles[2]>0.0) {
			if(sizes[0]>0.0) Nver = std::max(Nver,std::min(angles[2],sizes[0]));
			if(sizes[1]>0.0) Lver = std::max(Lver,std::min(angles[2],sizes[1]));
			if(sizes[2]>0.0) Hver = std::max(Hver,std::min(angles[2],sizes[2]));
			if(sizes[3]>0.0) Hver = std::max(Hver,std::min(angles[2],sizes[3]));
		}

		float Xpos = fabs(lines[i][0]+lines[i][2])/2.0;

		float Lside = mi_Xlocation_L(Xpos,width);
		float Mside = mi_Xlocation_M(Xpos,width);
		float Rside = mi_Xlocation_R(Xpos,width);

		if(Nver>Lver && Nver>Hver);//na pewno nie
		else {
			if(Lver>Hver) { //na pewno
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
			else { //byc moze
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
			
		}


		//horizontal

		float Nhor = 0.0;
		float Lhor = 0.0;
		float Hhor = 0.0;

		if(angles[0]>0.0) {
			if(sizes[0]>0.0) Lhor = std::max(Lhor,std::min(angles[0],sizes[0]));
			if(sizes[1]>0.0) Hhor = std::max(Hhor,std::min(angles[0],sizes[1]));
			if(sizes[2]>0.0) Hhor = std::max(Hhor,std::min(angles[0],sizes[2]));
			if(sizes[3]>0.0) Hhor = std::max(Hhor,std::min(angles[0],sizes[3]));
		}
		if(angles[1]>0.0) {
			if(sizes[0]>0.0) Lhor = std::max(Lhor,std::min(angles[1],sizes[0]));
			if(sizes[1]>0.0) Lhor = std::max(Lhor,std::min(angles[1],sizes[1]));
			if(sizes[2]>0.0) Lhor = std::max(Lhor,std::min(angles[1],sizes[2]));
			if(sizes[3]>0.0) Nhor = std::max(Nhor,std::min(angles[1],sizes[3]));
		} 
		if(angles[2]>0.0) {
			if(sizes[0]>0.0) Nhor = std::max(Nhor,std::min(angles[2],sizes[0]));
			if(sizes[1]>0.0) Nhor = std::max(Nhor,std::min(angles[2],sizes[1]));
			if(sizes[2]>0.0) Nhor = std::max(Nhor,std::min(angles[2],sizes[2]));
			if(sizes[3]>0.0) Nhor = std::max(Nhor,std::min(angles[2],sizes[3]));
		}

		float Ypos = fabs(lines[i][1]+lines[i][3])/2.0;

		float Hhei = mi_Ylocation_L(Ypos,height);
		float Mhei = mi_Ylocation_M(Ypos,height);
		float Lhei = mi_Ylocation_H(Ypos,height);

		if(Nhor>Lhor && Nhor>Hhor); //na pewno nie
		else {			
			if(Hhor>Lhor) { //na pewno
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
			else { //byc moze
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
		}
	}
}

bool LinesCornersFitting2::getIntersectionPoint(cv::Vec4i line1, cv::Vec4i line2, cv::Point2f &p) {
	
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

	if(p.y<0 || p.x<0 || p.y>=height || p.x>=width) return false;// std::cout<<"\n***\n"<<"intersection: "<<p.y<<"\n***\n";

	return true;

}

bool LinesCornersFitting2::get4intersectionPoints(cv::Point2f &p0, cv::Point2f &p1, cv::Point2f &p2, cv::Point2f &p3,
	std::vector<cv::Vec4i> lines, int ht, int vr, int hb, int vl) {
	return (getIntersectionPoint(lines[ht],lines[vr],p0) &&
		getIntersectionPoint(lines[vr],lines[hb],p1) &&
		getIntersectionPoint(lines[hb],lines[vl],p2) &&
		getIntersectionPoint(lines[vl],lines[ht],p3));
}

bool doorProportionTest(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2, cv::Point2f p3, float lower, float higher) {
	return (getLength(p3,p2)/getLength(p3,p0)>lower && getLength(p3,p2)/getLength(p3,p0)<higher &&
		getLength(p3,p2)/getLength(p2,p1)>lower && getLength(p3,p2)/getLength(p2,p1)<higher &&
		getLength(p0,p1)/getLength(p0,p3)>lower && getLength(p0,p1)/getLength(p0,p3)<higher &&
		getLength(p0,p1)/getLength(p2,p1)>lower && getLength(p0,p1)/getLength(p2,p1)<higher);
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

void LinesCornersFitting2::LinesCornersFitting2_processor() {

	std::vector<cv::Vec4i> lines = in_lines.read();
	std::vector<cv::Point2f> corners = in_corners.read();
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

	horizontal[0][0].insert(horizontal[0][0].end(),horizontal[1][0].begin(),horizontal[1][0].end());
	horizontal[0][1].insert(horizontal[0][1].end(),horizontal[1][1].begin(),horizontal[1][1].end());
	horizontal[2][0].insert(horizontal[2][0].end(),horizontal[1][0].begin(),horizontal[1][0].end());
	horizontal[2][1].insert(horizontal[2][1].end(),horizontal[1][1].begin(),horizontal[1][1].end());

	vertical[0][0].insert(vertical[0][0].end(),vertical[1][0].begin(),vertical[1][0].end());
	vertical[0][1].insert(vertical[0][1].end(),vertical[1][1].begin(),vertical[1][1].end());
	vertical[2][0].insert(vertical[2][0].end(),vertical[1][0].begin(),vertical[1][0].end());
	vertical[2][1].insert(vertical[2][1].end(),vertical[1][1].begin(),vertical[1][1].end());



	bool corners_check[lines.size()][lines.size()];
	for(int i=0; i<lines.size(); ++i) {
		for(int j=0; j<lines.size(); ++j) {
			corners_check[i][j] = false;
		}
	}
	for(int i=0; i<lines_pairs.size(); ++i) {
		corners_check[lines_pairs[i].first][lines_pairs[i].second] = true;
		corners_check[lines_pairs[i].second][lines_pairs[i].first] = true;
	}
	std::vector<std::vector<std::vector<cv::Vec4i > > > good_doors;
	//good_doors[quality][door_number][line_number]
	good_doors.resize(5);
	std::vector<std::vector<std::vector<cv::Point2f> > > good_corners;
	//good_corners[quality][door_number][corner_number]
	good_corners.resize(5);

	for(int ht=0; ht<horizontal[0][1].size(); ++ht) {
		int ht_line = horizontal[0][1][ht];
		for(int hb=0; hb<horizontal[2][1].size(); ++hb) {
			int hb_line = horizontal[2][1][hb];
			for(int vl=0; vl<vertical[0][1].size(); ++vl) {
				int vl_line = vertical[0][1][vl];
				for(int vr=0; vr<vertical[2][1].size(); ++vr) {
					int vr_line = vertical[2][1][vr];
					if((lines[ht_line][0]+lines[ht_line][2])/2>std::min(lines[vl_line][0],lines[vl_line][2]) &&
						(lines[ht_line][0]+lines[ht_line][2])/2<std::max(lines[vr_line][0],lines[vr_line][2]) && 
						(lines[hb_line][0]+lines[hb_line][2])/2>std::min(lines[vl_line][0],lines[vl_line][2]) &&
						(lines[hb_line][0]+lines[hb_line][2])/2<std::max(lines[vr_line][0],lines[vr_line][2])) {
							int q=0;
							cv::Point2f p0, p1, p2, p3;
							if(get4intersectionPoints(p0, p1, p2, p3, lines, ht_line, vr_line, hb_line, vl_line)) {
									if(doorProportionTest(p0, p1, p2, p3, size_ratio_min, size_ratio_max)) {
										if(getLength(lines[ht_line])/getLength(p3,p0)>0.3 &&
											getLength(lines[hb_line])/getLength(p2,p1)>0.3 &&
											getLength(lines[vr_line])/getLength(p0,p1)>0.3 &&
											getLength(lines[vl_line])/getLength(p3,p2)>0.3) {

												if(corners_check[ht_line][vr_line]) q++;
												if(corners_check[vr_line][hb_line]) q++;
												if(corners_check[hb_line][vl_line]) q++;
												if(corners_check[vl_line][ht_line]) q++;
												std::vector<cv::Vec4i> door;
												door.push_back(lines[ht_line]);
												door.push_back(lines[vr_line]);
												door.push_back(lines[hb_line]);
												door.push_back(lines[vl_line]);
												good_doors[q].push_back(door);
												std::vector<cv::Point2f> c;
												c.push_back(p0);
												c.push_back(p1);
												c.push_back(p2);
												c.push_back(p3);
												good_corners[q].push_back(c);
										}
									}
								
							
							}
					}
				}
			}
		}
	}

	int best_quality=4;
	for(best_quality=4; good_doors[best_quality].size()==0 && best_quality>0; --best_quality);
	//std::cout<<"\n *** best quality: "<<best_quality<<", doors: "<<good_doors[best_quality].size()<<" ***\n";
	

	if(good_doors[best_quality].size()==0 || best_quality==0) {
		std::vector<std::vector<std::vector<cv::Vec4i > > > good_doors2;
		good_doors2.resize(5);
		std::vector<std::vector<std::vector<cv::Point2f> > > good_corners2;
		good_corners2.resize(5);

		horizontal[0][1].insert(horizontal[0][1].end(),horizontal[0][0].begin(),horizontal[0][0].end());
		horizontal[2][1].insert(horizontal[2][1].end(),horizontal[2][0].begin(),horizontal[2][0].end());
		vertical[0][1].insert(vertical[0][1].end(),vertical[0][0].begin(),vertical[0][0].end());
		vertical[2][1].insert(vertical[2][1].end(),vertical[2][0].begin(),vertical[2][0].end());
		for(int ht=0; ht<horizontal[0][1].size(); ++ht) {
		int ht_line = horizontal[0][1][ht];
		for(int hb=0; hb<horizontal[2][1].size(); ++hb) {
			int hb_line = horizontal[2][1][hb];
			for(int vl=0; vl<vertical[0][1].size(); ++vl) {
				int vl_line = vertical[0][1][vl];
				for(int vr=0; vr<vertical[2][1].size(); ++vr) {
					int vr_line = vertical[2][1][vr];
					if((lines[ht_line][0]+lines[ht_line][2])/2>std::min(lines[vl_line][0],lines[vl_line][2]) &&
						(lines[ht_line][0]+lines[ht_line][2])/2<std::max(lines[vr_line][0],lines[vr_line][2]) && 
						(lines[hb_line][0]+lines[hb_line][2])/2>std::min(lines[vl_line][0],lines[vl_line][2]) &&
						(lines[hb_line][0]+lines[hb_line][2])/2<std::max(lines[vr_line][0],lines[vr_line][2])) {
							int q=0;
							cv::Point2f p0, p1, p2, p3;
							if(get4intersectionPoints(p0, p1, p2, p3, lines, ht_line, vr_line, hb_line, vl_line)) {
								if(doorProportionTest(p0, p1, p2, p3, size_ratio_min, size_ratio_max)) {

									if(getLength(lines[ht_line])/getLength(p3,p0)>0.3 &&
										getLength(lines[hb_line])/getLength(p2,p1)>0.3 &&
										getLength(lines[vr_line])/getLength(p0,p1)>0.3 &&
										getLength(lines[vl_line])/getLength(p3,p2)>0.3) {

										if(corners_check[ht_line][vr_line]) q++;
										if(corners_check[vr_line][hb_line]) q++;
										if(corners_check[hb_line][vl_line]) q++;
										if(corners_check[vl_line][ht_line]) q++;
										std::vector<cv::Vec4i> door;
										door.push_back(lines[ht_line]);
										door.push_back(lines[vr_line]);
										door.push_back(lines[hb_line]);
										door.push_back(lines[vl_line]);
										good_doors2[q].push_back(door);
										std::vector<cv::Point2f> c;
										c.push_back(p0);
										c.push_back(p1);
										c.push_back(p2);
										c.push_back(p3);
										good_corners2[q].push_back(c);
									}

									
								}		
							}					
						}
					}
				}
			}
		}
		int best_quality2;
		for(best_quality2=4; good_doors2[best_quality2].size()==0 && best_quality2>0; --best_quality2);
		//std::cout<<"\n *** best quality 2: "<<best_quality2<<", doors: "<<good_doors2[best_quality2].size()<<" ***\n";
		if(good_doors[best_quality].size()==0) {
			good_doors = good_doors2;
			good_corners = good_corners2;
			best_quality = best_quality2;
		}
		else {
			good_doors[best_quality+1] = good_doors[best_quality];
			good_doors[best_quality] = good_doors2[best_quality2];
			good_corners[best_quality+1] = good_corners[best_quality];
			good_corners[best_quality] = good_corners2[best_quality2];
			best_quality = best_quality + 1;
		}
	}

	if(best_quality>0) {
		good_doors[best_quality].insert(good_doors[best_quality].end(),good_doors[best_quality-1].begin(),good_doors[best_quality-1].end());
		good_corners[best_quality].insert(good_corners[best_quality].end(),good_corners[best_quality-1].begin(),good_corners[best_quality-1].end());
	}

	std::vector<std::vector<int> > door_groups;
	if(good_doors[best_quality].size()>1) {
		std::vector<cv::Mat> shapes;
		for(int i=0; i<good_doors[best_quality].size(); ++i) {
			cv::Mat shape = cv::Mat::zeros(img.rows,img.cols,CV_8U);
			cv::Point mask_points[1][4];
			mask_points[0][0] = cv::Point(good_corners[best_quality][i][0]);
			mask_points[0][1] = cv::Point(good_corners[best_quality][i][1]);
			mask_points[0][2] = cv::Point(good_corners[best_quality][i][2]);
			mask_points[0][3] = cv::Point(good_corners[best_quality][i][3]);
			const cv::Point* ppt[1] = {mask_points[0]};
			int npt[] = {4};
			cv::fillPoly(shape,ppt,npt,1,cv::Scalar(255,255,255));
			shapes.push_back(shape.clone());
		}
		float diffs[shapes.size()][shapes.size()];
		for(int i=0; i<shapes.size()-1; ++i) {
			for(int j=i+1; j<shapes.size(); ++j) {
				cv::Mat diff_img;
				cv::bitwise_and(shapes[i],shapes[j],diff_img);
				int diff_number = cv::countNonZero(diff_img);
				int bigger_number = std::max(cv::countNonZero(shapes[i]),cv::countNonZero(shapes[j]));
				diffs[i][j] = float(diff_number)/float(bigger_number);
				diffs[j][i] = float(diff_number)/float(bigger_number);
			}
			diffs[i][i] = -1.0;
		}
		diffs[shapes.size()-1][shapes.size()-1]=0;

		
		for(int i=0; i<good_doors[best_quality].size()-1; ++i) {
			std::vector<int> group;
			group.push_back(i);
			for(int j=i+1; j<good_doors[best_quality].size(); ++j) {
				bool add = true;
				for(int k=0; k<group.size(); ++k) {
					if(diffs[i][j]<0.95) {
						add = false;
						break;
					}
				}
				if(add) {
					group.push_back(j);
				}
			}
			door_groups.push_back(group);
		}
		



		
		
	}
	else if(good_doors[best_quality].size()==1){
		std::vector<int> group;
		group.push_back(0);
		door_groups.push_back(group);
	}

	int max_group=0;
	for(int i=1; i<door_groups.size(); ++i) {
		if(door_groups[i].size()>=door_groups[max_group].size()) max_group=i;
	}

	//tutaj sprawdzac wariancje z barwy

	int best_group=0;
	std::vector<float> devs;
	cv::Mat mask, hsv, hsv_split[3];
	cv::cvtColor(img,hsv,CV_BGR2HSV);
	cv::split(hsv, hsv_split);
	for(int g=0; g<door_groups.size(); ++g) {
		float dev_sum = 0.0;
		for(int d=0; d<door_groups[g].size(); ++d) {
			mask = cv::Mat::zeros(img.rows,img.cols,CV_8U);
			/*std::cout<<"\n\n"<<"best_quality="<<best_quality<<", <"<<good_corners.size()<<"\n";
			std::cout<<"g="<<g<<", <"<<door_groups.size()<<"\n";
			std::cout<<"d="<<d<<", <"<<door_groups[g].size()<<"\n";
			std::cout<<"door_groups[g][d]="<<door_groups[g][d]<<", <"<<good_corners[best_quality].size()<<"\n\n\n";*/
			drawPoly(mask,good_corners[best_quality][door_groups[g][d]], cv::Scalar(255,255,255));
			cv::Scalar mean, dev;
			cv::meanStdDev(hsv_split[hsv_channel],mean,dev,mask);
			dev_sum += dev[0];
		}
		dev_sum /= float(door_groups[g].size());
		devs.push_back(dev_sum);
	}
	for(int i=1; i<devs.size(); ++i) {
		if(devs[i]<devs[best_group]) best_group=i;
	}

	//********************

	if(dev_choice) max_group = best_group;
	std::vector<cv::Point2f> final_door;
	if(good_doors[best_quality].size()>0)
		{
			final_door = good_corners[best_quality][door_groups[max_group][0]];


			for(int i=1; i<door_groups[max_group].size(); ++i) {
				final_door[0].x += good_corners[best_quality][door_groups[max_group][i]][0].x;
				final_door[0].y += good_corners[best_quality][door_groups[max_group][i]][0].y;
				final_door[1].x += good_corners[best_quality][door_groups[max_group][i]][1].x;
				final_door[1].y += good_corners[best_quality][door_groups[max_group][i]][1].y;
				final_door[2].x += good_corners[best_quality][door_groups[max_group][i]][2].x;
				final_door[2].y += good_corners[best_quality][door_groups[max_group][i]][2].y;
				final_door[3].x += good_corners[best_quality][door_groups[max_group][i]][3].x;
				final_door[3].y += good_corners[best_quality][door_groups[max_group][i]][3].y;
				
			}
			
			final_door[0].x /= door_groups[max_group].size();
			final_door[0].y /= door_groups[max_group].size();
			final_door[1].x /= door_groups[max_group].size();
			final_door[1].y /= door_groups[max_group].size();
			final_door[2].x /= door_groups[max_group].size();
			final_door[2].y /= door_groups[max_group].size();
			final_door[3].x /= door_groups[max_group].size();
			final_door[3].y /= door_groups[max_group].size();


			cv::line(img,final_door[0],final_door[1],cv::Scalar(255,0,255),10);
			cv::line(img,final_door[1],final_door[2],cv::Scalar(255,0,255),10);
			cv::line(img,final_door[2],final_door[3],cv::Scalar(255,0,255),10);
			cv::line(img,final_door[3],final_door[0],cv::Scalar(255,0,255),10);

			cv::putText(img, "0", final_door[0],cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0,0,255),3);
			cv::putText(img, "1", final_door[1],cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0,0,255),3);
			cv::putText(img, "2", final_door[2],cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0,0,255),3);
			cv::putText(img, "3", final_door[3],cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0,0,255),3);

			//std::cout<<"\n***\n"<<final_door[0]<<" "<<final_door[1]<<" "<<final_door[2]<<" "<<final_door[3]<<"\n***\n";



			//dwa sprawdzenia:
			//1. podobna (mala) wariancja (np. posortowac wszesniej i znalexc miejsce skoku)
			//2. wsrod tych z podobna wariancja wyrzucic te, ktore maja rozne pola	

			/*if(good_corners[best_quality].size()>0) {
				int idx = prop;
				if(idx>=good_corners[best_quality].size()) idx = good_corners[best_quality].size()-1;

				cv::Scalar color = cv::Scalar(0,0,255);
				cv::line(img,good_corners[best_quality][idx][0],good_corners[best_quality][idx][1],color,3);
				cv::line(img,good_corners[best_quality][idx][1],good_corners[best_quality][idx][2],color,3);
				cv::line(img,good_corners[best_quality][idx][2],good_corners[best_quality][idx][3],color,3);
				cv::line(img,good_corners[best_quality][idx][3],good_corners[best_quality][idx][0],color,3);

				color = cv::Scalar(255,0,0);
				if(door_groups.size()>0) {
					for(int i=0; i<door_groups[max_group].size(); ++i) {
						cv::line(img,good_corners[best_quality][door_groups[max_group][i]][0],good_corners[best_quality][door_groups[max_group][i]][1],color,3);
						cv::line(img,good_corners[best_quality][door_groups[max_group][i]][1],good_corners[best_quality][door_groups[max_group][i]][2],color,3);
						cv::line(img,good_corners[best_quality][door_groups[max_group][i]][2],good_corners[best_quality][door_groups[max_group][i]][3],color,3);
						cv::line(img,good_corners[best_quality][door_groups[max_group][i]][3],good_corners[best_quality][door_groups[max_group][i]][0],color,3);
					}
				}
				

				
			}*/
			/*if(good_doors[best_quality].size()>0) {
				int idx;
				if(best_door>-1) idx = best_door;
				else idx = prop;
				if(idx>=good_doors[best_quality].size()) idx = good_doors[best_quality].size()-1;
				for(int j=0; j<good_doors[best_quality][idx].size(); ++j) {
					cv::Point p1 = cv::Point(good_doors[best_quality][idx][j][0],good_doors[best_quality][idx][j][1]);
					cv::Point p2 = cv::Point(good_doors[best_quality][idx][j][2],good_doors[best_quality][idx][j][3]);
					cv::line(img,p1,p2,cv::Scalar(0,255,0),3);
				}
			}*/


			for(int side=0; side<vertical.size(); ++side) {
				for(int prob1=0; prob1<vertical[side].size(); ++prob1) {
					for(int l=0; l<vertical[side][prob1].size(); ++l) {
						int idx = vertical[side][prob1][l];
						cv::Point p1 = cv::Point(lines[idx][0],lines[idx][1]);
						cv::Point p2 = cv::Point(lines[idx][2],lines[idx][3]);
						cv::Point p3 = cv::Point((lines[idx][0]+lines[idx][2])/2,(lines[idx][1]+lines[idx][3])/2);
						cv::line(img,p1,p2,cv::Scalar(0,255,0),3);
						if(side==0) {
							if(prob1==0) cv::putText(img,"probV-Left",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
							if(prob1==1) cv::putText(img,"sureV-Left",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
						}
						else if(side==1) {
							if(prob1==0) cv::putText(img,"probV-Mid",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
							if(prob1==1) cv::putText(img,"sureV-Mid",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
						}
						else if(side==2) {
							if(prob1==0) cv::putText(img,"probV-Right",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
							if(prob1==1) cv::putText(img,"sureV-Right",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
						}
					}
				}
			}

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
							if(prob1==1) cv::putText(img,"sureH-High",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
						}
						else if(hei==1) {
							if(prob1==0) cv::putText(img,"probH-Mid",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
							if(prob1==1) cv::putText(img,"sureH-Mid",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
						}
						else if(hei==2) {
							if(prob1==0) cv::putText(img,"probH-Low",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
							if(prob1==1) cv::putText(img,"sureH-Low",p3,cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255));
						}
					}
				}
			}
		}

	std::vector<std::vector<float> > doorVec;
	for(int i=0; i<final_door.size(); ++i) {
		std::vector<float> p;
		p.push_back(final_door[i].x);
		p.push_back(final_door[i].y);
		doorVec.push_back(p);
	}

	//std::cout<<"\n\n*** width: "<<width<<" , height: "<<height<<" *** \n\n";
	out_img.write(img);
	out_door.write(final_door);
	out_doorVec.write(doorVec);

	

	
}



} //: namespace LinesCornersFitting2
} //: namespace Processors
