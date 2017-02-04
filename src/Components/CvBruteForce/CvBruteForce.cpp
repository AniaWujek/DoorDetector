/*!
 * \file
 * \brief
 * \author Jan Figat
 * \e-mail jan.figat@gmail.com
 */

#include <memory>
#include <string>

#include "CvBruteForce.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CvBruteForce {

CvBruteForce::CvBruteForce(const std::string & name) :
		Base::Component(name),
		distance_recalc("recalculate_distance", true),
		print_stats("print_stats", true),
		dist("distance", 0.15),
		model_number("model_number", -1)
{
	registerProperty(distance_recalc);
	registerProperty(print_stats);
	registerProperty(dist);

	model_number.addConstraint("-1");
	model_number.addConstraint("3");
	registerProperty(model_number);
}

CvBruteForce::~CvBruteForce() {
}

void CvBruteForce::prepareInterface() {
	// Register handlers with their dependencies.
	registerHandler("onNewImage", boost::bind(&CvBruteForce::onNewImage, this));
	addDependency("onNewImage", &in_features);
	addDependency("onNewImage", &in_featuresVec);
	addDependency("onNewImage", &in_descriptors);
	addDependency("onNewImage", &in_descriptorsVec);
	addDependency("onNewImage", &in_rectVec);

	// Input and output data streams.
	registerStream("in_features", &in_features);
	registerStream("in_featuresVec", &in_featuresVec);
	registerStream("in_descriptors", &in_descriptors);
	registerStream("in_descriptorsVec", &in_descriptorsVec);
	registerStream("out_matches", &out_matches);
	registerStream("out_features", &out_features);
	registerStream("out_descriptors", &out_descriptors);
	registerStream("in_rectVec", &in_rectVec);
	registerStream("out_rect", &out_rect);
}

bool CvBruteForce::onInit() {

	return true;
}

bool CvBruteForce::onFinish() {
	return true;
}

bool CvBruteForce::onStop() {
	return true;
}

bool CvBruteForce::onStart() {
	return true;
}


void CvBruteForce::onNewImage()
{
	CLOG(LTRACE) << "CvBruteForce::onNewImage\n";
	try {
		// Read input features.
		Types::Features features = in_features.read();
		std::vector<Types::Features> featuresVec = in_featuresVec.read();
		// Read input descriptors.
		cv::Mat descriptors = in_descriptors.read();
		std::vector<cv::Mat> descriptorsVec = in_descriptorsVec.read();

		// Matching descriptor vectors using BruteForce matcher.
		BFMatcher matcher(cv::NORM_HAMMING); //BruteForce
		std::vector<std::vector< DMatch > > matchesVec;
		for(int i=0; i<descriptorsVec.size(); ++i) {
			std::vector<DMatch> matches;
			matcher.match( descriptorsVec[i], descriptors, matches );
			matchesVec.push_back(matches);
		}
		
		std::vector<double> distVec;
		distVec.resize(descriptorsVec.size());
		for(int i=0; i<distVec.size(); ++i) distVec[i] = dist;
		if (distance_recalc) {
			//-- Quick calculation of max and min distances between keypoints.
			for(int j=0; j<descriptorsVec.size(); ++j) {
				double max_dist = 0;
				double min_dist = 100;
				for( int i = 0; i < descriptorsVec[j].rows; i++ )
				{
					double dist = matchesVec[j][i].distance;
					if( dist < min_dist ) min_dist = dist;
					if( dist > max_dist ) max_dist = dist;
				}
				distVec[j] = 2*min_dist;
				CLOG(LINFO) << " Max dist : " << (double)max_dist;
				CLOG(LINFO) << " Min dist : " << (double)min_dist;
				CLOG(LINFO) << " Dist : " << (double)dist << std::endl;
			}
		}

		//Draw only "good" matches (i.e. whose distance is less than 2*min_dist ).
		//PS.- radiusMatch can also be used here.
		std::vector< std::vector< DMatch > > good_matchesVec;
		good_matchesVec.resize(descriptorsVec.size());
		for(int j=0; j<matchesVec.size(); ++j) {
			for( int i = 0; i < descriptorsVec[j].rows; i++ )
			{
				if( matchesVec[j][i].distance < distVec[j] )
					good_matchesVec[j].push_back( matchesVec[j][i]);
			}
		}

		int best_matches = 0;
		for(int i=1; i<good_matchesVec.size(); ++i) {
			if(float(good_matchesVec[i].size())/float(matchesVec[i].size()) > float(good_matchesVec[best_matches].size())/float(matchesVec[best_matches].size())) best_matches = i;
		}

		CLOG(LNOTICE) << "Best model: " << best_matches;

		// Print stats.
		/*if (print_stats) {
			for( int i = 0; i < good_matches.size(); i++ )
			{
				CLOG(LINFO) << " Good Match [" << i <<"] Keypoint 1: " << good_matches[i].queryIdx << "  -- Keypoint 2: " << good_matches[i].trainIdx;
			}
			CLOG(LINFO) << std::endl;		}*/


		// Write the result to the output.
		if(model_number>=0) best_matches = model_number;

		std::vector<std::vector<cv::Point2f> > rects = in_rectVec.read();


		out_matches.write(good_matchesVec[best_matches]);
		out_features.write(featuresVec[best_matches]);
		out_descriptors.write(descriptorsVec[best_matches]);
		out_rect.write(rects[best_matches]);
	} catch (...) {
		CLOG(LERROR) << "CvBruteForce::onNewImage failed\n";
	}
}



} //: namespace CvBruteForce
} //: namespace Processors
