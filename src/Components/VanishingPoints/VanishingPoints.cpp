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

void VanishingPoints::VanishingPointsProcessor() {

	std::vector<cv::Vec4i> lines = in_lines.read();

	std::vector<std::pair<cv::Vec4i,cv::Vec4i> > samples(hypotheses);

	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<> dis(0, lines.size()-1);

	for(int i=0; i<hypotheses; ++i) {
		samples[i] = std::make_pair(lines[dis(gen)],lines[dis(gen)]);
	}

	std::vector<cv::Point2f> vanishing_points_hypotheses(hypotheses);
	for(int i=0; i<hypotheses; ++i) {
		cv::Point2f v;
		if(getIntersectionPoint(samples[i].first,samples[i].second,v)) {
			vanishing_points_hypotheses[i] = v;
		}
		std::cout<<"\n";
	}

	bool preference_matrix [lines.size()][int(hypotheses)];


	




}



} //: namespace VanishingPoints
} //: namespace Processors
