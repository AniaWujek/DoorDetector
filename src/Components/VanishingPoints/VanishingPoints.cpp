/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "VanishingPoints.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <random>
#include "Types/DrawableContainer.hpp"
#include "Types/Line.hpp"

namespace Processors {
namespace VanishingPoints {

VanishingPoints::VanishingPoints(const std::string & name) :
		Base::Component(name) , 
		vanishing_points("vanishing_points", 3), 
		consensus_threshold("consensus_threshold", 2), 
		hypotheses("hypotheses", 500) {
	registerProperty(vanishing_points);
	registerProperty(consensus_threshold);
	registerProperty(hypotheses);

}

VanishingPoints::~VanishingPoints() {
}

void VanishingPoints::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_lines", &in_lines);
	registerStream("out_linesVecs", &out_linesVecs);
	registerStream("out_vanishingPoints", &out_vanishingPoints);
	registerStream("out_linesDrawable", &out_linesDrawable);
	// Register handlers
	registerHandler("VanishingPointsProcessor", boost::bind(&VanishingPoints::VanishingPointsProcessor, this));
	addDependency("VanishingPointsProcessor", &in_lines);

}

bool VanishingPoints::onInit() {

	return true;
}

bool VanishingPoints::onFinish() {
	return true;
}

bool VanishingPoints::onStop() {
	return true;
}

bool VanishingPoints::onStart() {
	return true;
}

bool getIntersectionPoint(cv::Vec4i line1, cv::Vec4i line2, cv::Point2f &p) {
	cv::Point2f x = cv::Point2f(line2[0],line2[1])-cv::Point2f(line1[0],line1[1]);
	cv::Point2f d1 = cv::Point2f(line1[2],line1[3])-cv::Point2f(line1[0],line1[1]);
	cv::Point2f d2 = cv::Point2f(line2[2],line2[3])-cv::Point2f(line2[0],line2[1]);

	float cross = d1.x*d2.y - d1.y*d2.x;
	if(fabs(cross) < 1e-8) return false;

	double t1 = (x.x*d2.y - x.y*d2.x)/cross;
	p = cv::Point2f(line1[0],line1[1]) + d1*t1;
	return true;

}

float getSquareDistance(cv::Point2f p0, cv::Point2f p1) {
	return (p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y);
}

bool areConsistent(cv::Vec4i line, cv::Point2f v_point, int threshold) {
	cv::Point2f centroid = cv::Point2f((line[0]+line[2])/2.0,(line[1]+line[3])/2.0);
	float x1 = centroid.x;
	float y1 = centroid.y;

	float x2 = v_point.x;
	float y2 = v_point.y;

	float dist0 = getSquareDistance(cv::Point2f(line[0],line[1]),v_point);
	float dist1 = getSquareDistance(cv::Point2f(line[2],line[3]),v_point);

	float x,y;
	if(dist0 > dist1) {
		x = line[0];
		y = line[1];
	}
	else {
		x = line[2];
		y = line[3];
	}

	float A = x - x1;
	float B = y - y1;
	float C = x2 - x1;
	float D = y2 - y1;

	float distance = fabs(A*D-C*B)/sqrt(C*C+D*D);

	if(distance <= threshold) return true;
	
	return false;
}

float jaccardDistance(std::vector<bool> &set1, std::vector<bool> &set2) {

	int sum = 0;
	int intersection = 0;
	for(int i=0; i<set1.size(); ++i) {
		if(set1[i] || set2[i]) {
			sum++;
			if(set1[i] && set2[i]) intersection++;
		}
	}
	float sumf = float(sum);
	float intersectionf = float(intersection);
	return (sumf - intersectionf)/sumf;
}

void mergeClusters(std::vector<std::vector<bool> > &clusters, int c1, int c2) {
	for(int i=0; i<clusters[c1].size(); ++i) {
		clusters[c1][i] = clusters[c1][i] || clusters[c2][i];
	}
	clusters.erase(clusters.begin()+c2);
}

void getVPoints(std::vector<cv::Point2f> &vanishing_points_hypotheses,
	std::vector<cv::Vec4i> &lines) {

	int hypotheses = vanishing_points_hypotheses.size();

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(0, lines.size()-1);

	for(int i=0; i<hypotheses; ++i) {
		cv::Point2f v;
		int l1 = dis(gen);
		int l2 = dis(gen);
		std::pair<cv::Vec4i,cv::Vec4i> sample = std::make_pair(lines[l1],lines[l2]);
		while(!getIntersectionPoint(sample.first,sample.second,v)) {
			l1 = dis(gen);
			l2 = dis(gen);
			sample = std::make_pair(lines[l1],lines[l2]);
		}
		vanishing_points_hypotheses[i] = v;
	}

}

void VanishingPoints::VanishingPointsProcessor() {

	std::vector<cv::Vec4i> lines = in_lines.read();	

	std::vector<cv::Point2f> vanishing_points_hypotheses(hypotheses);

	getVPoints(vanishing_points_hypotheses, lines);
	

	std::vector<std::vector<bool> > clusters(lines.size());

	for(int i=0; i<lines.size(); ++i) {
		for(int j=0; j<int(hypotheses); ++j) {
			clusters[i].push_back(areConsistent(lines[i],vanishing_points_hypotheses[j],consensus_threshold));
		}
	}

	std::vector<std::vector<float> > jaccardDists(clusters.size());

	//uwaga, drugi indeks musi byc mniejszy
	float minJD = 2.0;
	int min1, min2;
	for(int j=0; j<clusters.size()-1; ++j) {
		for(int i=j+1; i<clusters.size(); ++i) {
			jaccardDists[i].push_back(jaccardDistance(clusters[i],clusters[j]));
			if(jaccardDists[i][j] < minJD) {
				minJD = jaccardDists[i][j];
				min1 = i;
				min2 = j;
			}
		}
	}

	while(minJD < 0.9 && clusters.size()>2) {
		mergeClusters(clusters, min2, min1);
		jaccardDists.erase(jaccardDists.begin()+min1);
		for(int i=0; i<jaccardDists.size(); ++i) {
			if(i>min2) {
				jaccardDists[i][min2] = jaccardDistance(clusters[i],clusters[min2]);
			}
			if(i>=min1) {
				jaccardDists[i].erase(jaccardDists[i].begin()+min1);
			}
		}
		minJD = 2.0;
		for(int i=0; i<jaccardDists.size(); ++i) {
			for(int j=0; j<jaccardDists[i].size(); ++j) {
				if(jaccardDists[i][j]<minJD) {
					minJD = jaccardDists[i][j];
					min1 = i;
					min2 = j;
				}
			}
		}
	}
	
	std::cout<<"**********\n\n"<<clusters.size()<<" "<<jaccardDistance(clusters[0],clusters[1])<<"\n\n********";

	Types::DrawableContainer c;
	std::vector<cv::Scalar> colors{cv::Scalar(0,0,255),cv::Scalar(0,128,255),cv::Scalar(0,255,255),cv::Scalar(0,255,128),cv::Scalar(255,128,0),cv::Scalar(0,0,255),cv::Scalar(255,0,127),
		cv::Scalar(255,0,255),cv::Scalar(0,102,0),cv::Scalar(204,204,0)};
	for(int i=0; i<clusters.size(); ++i) {
		for(int j=0; j<clusters[i].size(); ++j) {
			if(clusters[i][j]) {
				cv::Scalar color;
				if(i < colors.size()) {
					color = colors[i];
				} 
				else {
					color = cv::Scalar(0,0,0);
				}
				/*cv::Vec4i line2add = lines[]
				c.add(new Types::Line(cv::Point()))*/
			}
			
		}
	}

}



} //: namespace VanishingPoints
} //: namespace Processors
