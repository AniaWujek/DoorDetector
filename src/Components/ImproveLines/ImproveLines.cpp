/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "ImproveLines.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <algorithm>

#include <iostream>
#include <fstream>

namespace Processors {
namespace ImproveLines {

ImproveLines::ImproveLines(const std::string & name) :
		Base::Component(name) ,
		collinearRatio("collinearRatio",0.6),
		shortRatio("shortRatio",0.2),
		closeRatio("closeRatio",0.9) {
	registerProperty(collinearRatio);
	registerProperty(shortRatio);
	registerProperty(closeRatio);
}

ImproveLines::~ImproveLines() {
}

void ImproveLines::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_lines", &in_lines);
	registerStream("in_img", &in_img);
	registerStream("out_lines", &out_lines);
	registerStream("out_linesDrawable", &out_linesDrawable);
	// Register handlers
	registerHandler("improveLinesProcessor", boost::bind(&ImproveLines::improveLinesProcessor, this));
	addDependency("improveLinesProcessor", &in_lines);
	addDependency("improveLinesProcessor", &in_img);

}

bool ImproveLines::onInit() {


	return true;
}

bool ImproveLines::onFinish() {
	return true;
}

bool ImproveLines::onStop() {
	return true;
}

bool ImproveLines::onStart() {

	return true;
}

// zakladajac, ze linia jest skierowana w gore, czyli kat [0,pi]
float getAngle(cv::Vec4i line) {
	float x, y;
	x = line[2] - line[0];
	y = -line[3] + line[1];
	return atan2(y,x);
}

std::vector<float> getAngles(std::vector<cv::Vec4i> &lines) {
	std::vector<float> angles;
	float theta;
	for(int i = 0; i < lines.size(); ++i) {

		//zeby linie byly skierowane w gore
		if(lines[i][3]>lines[i][1]) {
			lines[i] = cv::Vec4i(lines[i][2],lines[i][3],lines[i][0],lines[i][1]);
		}

		theta = getAngle(lines[i]);		

		angles.push_back(theta);
	}

	return angles;
}

float getTriangleArea(float x0, float y0, float x1, float y1, float x2, float y2) {
	return fabs(((x0-x2)*(y1-y0)-(x0-x1)*(y2-y0))/2.0);
}

float getLength(cv::Point p0, cv::Point p1) {
    return sqrt((p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y));
}
float getLength(cv::Vec4i line) {
	return getLength(cv::Point(line[0],line[1]),cv::Point(line[2],line[3]));
}

bool areCollinear(cv::Vec4i line0, cv::Vec4i line1, float collinearRatio) {
	float area = getTriangleArea(line0[0],line0[1],line0[2],line0[3],line1[0],line1[1])+
					getTriangleArea(line0[0],line0[1],line0[2],line0[3],line1[2],line1[3])+
					getTriangleArea(line1[0],line1[1],line1[2],line1[3],line0[0],line0[1])+
					getTriangleArea(line1[0],line1[1],line1[2],line1[3],line0[2],line0[3]);
	area /= 2.0;
	float length = (getLength(line0)+getLength(line1))*(getLength(line0)+getLength(line1));
	bool collinear = area/length<collinearRatio;

}

cv::Vec4i findLongestLine(vector<cv::Point> &points) {
	float maxDist = getLength(points[0],points[1]);
	int max0 = 0;
	int max1 = 1;
	for(int i = 0; i < points.size()-1; ++i) {
		for(int j = i+1; j < points.size(); ++j) {
			float dist = getLength(points[i],points[j]);
			if(dist > maxDist) {
				maxDist = dist;
				if(points[i].y > points[j].y) {
					max0 = i;
					max1 = j;
				}
				else {
					max0 = j;
					max1 = i;
				}
			}
		} 
	}
	cv::Vec4i line = cv::Vec4i(points[max0].x,points[max0].y,points[max1].x,points[max1].y);
	return line;
}



bool areClose(cv::Vec4i line0, cv::Vec4i line1, float closeRatio) {
	float connectorLength = getLength(cv::Point((line0[0]+line0[2])/2.0,(line0[1]+line0[3])/2.0),
										cv::Point((line1[0]+line1[2])/2.0,(line1[1]+line1[3])/2.0));
	if(connectorLength < closeRatio*(getLength(line0)+getLength(line1))) return true;
}

void connectLines(std::vector<cv::Vec4i> &lines, std::vector<float> &angles, float collinearRatio, float closeRatio) {
	//connect lines
	
	bool done = false;
	while(!done) {
		done = true;
		for(int l1 = 0; l1 < lines.size()-1; ++l1) {
			vector<int> to_connect;
			to_connect.push_back(l1);
			for(int l2 = l1+1; l2 < lines.size(); ++l2) {
				bool collinear = true;
				int c = 0;
				while(collinear && c < to_connect.size()) {
					collinear = (fabs(angles[l2] - angles[to_connect[c]])<0.05 || M_PI-fabs(angles[l2] - angles[to_connect[c]])<0.1)
						&& areCollinear(lines[to_connect[c]],lines[l2], collinearRatio)
						&& areClose(lines[to_connect[c]],lines[l2], closeRatio);
					c++;
				}
				if(collinear) {
					to_connect.push_back(l2);
				}
			}//end for l2
			if(to_connect.size()>1) {
				done = false;
				std::vector<cv::Point> points;
				for(int c = 0; c < to_connect.size(); ++c) {
					points.push_back(cv::Point(lines[to_connect[c]][0],lines[to_connect[c]][1]));
					points.push_back(cv::Point(lines[to_connect[c]][2],lines[to_connect[c]][3]));
				}
				lines[l1] = findLongestLine(points);
				angles[l1] = getAngle(lines[l1]);

				for(int c = to_connect.size()-1; c > 0; --c) {
					lines.erase(lines.begin()+to_connect[c]);
					angles.erase(angles.begin()+to_connect[c]);
				}

			}
		}// end for l1

	}// end while

}

void removeShort(std::vector<cv::Vec4i> &lines, int rows, int cols, float shortRatio) {
	float thresh = float(min(cols, rows))*shortRatio;
	for(int i = lines.size()-1; i >= 0; --i) {
		if(getLength(lines[i]) < thresh) {
			lines.erase(lines.begin()+i);
		}
	}

}

void ImproveLines::improveLinesProcessor() {

	std::vector<cv::Vec4i> lines = in_lines.read();
	cv::Mat img = in_img.read().clone();
	std::vector<float> angles = getAngles(lines);

	connectLines(lines,angles, collinearRatio, closeRatio);
	removeShort(lines, img.rows, img.cols, shortRatio);	

	Types::DrawableContainer c;
	for( size_t i = 0; i < lines.size(); i++ )
	{
        c.add(new Types::Line(cv::Point(lines[i][0], lines[i][1]),
            cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,0)));		
	}

	out_lines.write(lines);
	out_linesDrawable.write(c);


	

}



} //: namespace ImproveLines
} //: namespace Processors
