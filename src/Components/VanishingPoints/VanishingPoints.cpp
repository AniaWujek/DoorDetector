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
		hypotheses("hypotheses", 100),
		jaccard_thresh("jaccard_thresh", 0.7),
		auto_hypotheses("auto_hypotheses", 1) {
	registerProperty(vanishing_points);
	registerProperty(consensus_threshold);

	hypotheses.addConstraint("50");
	hypotheses.addConstraint("500");
	registerProperty(hypotheses);

	registerProperty(jaccard_thresh);
	registerProperty(auto_hypotheses);

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

bool pointClose2Line(cv::Vec4i line, cv::Point2f p) {
	int minX, minY, maxX, maxY;
	minX = std::min(line[0],line[2]);
	minY = std::min(line[1],line[3]);
	maxX = std::max(line[0],line[2]);
	maxY = std::max(line[1],line[3]);

	if(p.x>=minX && p.x<=maxX && p.y>=minY && p.y<=maxY) return true;
	else return false;
}

bool getIntersectionPoint(cv::Vec4i line1, cv::Vec4i line2, cv::Point2f &p) {
	
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


	//if(pointClose2Line(line1, p) || pointClose2Line(line2, p)) return false;
	if(p.x>=0 && p.x<2304 && p.y>=0 && p.y<1728) return false;

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

void getVPoints(std::vector<cv::Point2f> &vanishing_points_hypotheses, std::vector<cv::Vec4i> &lines) {

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

	int h_size = hypotheses;
	if(auto_hypotheses) {
		h_size = 10*lines.size();
		std::cout<<"\n*** "<<h_size<<" ***\n";
	}

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

	std::vector<std::vector<cv::Vec4i> > linesClusters;
	for(int i=0; i<lines.size(); ++i) {
		std::vector<cv::Vec4i> linesVec = {lines[i]};
		linesClusters.push_back(linesVec);
	}

	while(minJD < jaccard_thresh && clusters.size()>2) {
		mergeClusters(clusters, min2, min1);
		linesClusters[min2].insert(linesClusters[min2].end(),linesClusters[min1].begin(),linesClusters[min1].end());
		linesClusters.erase(linesClusters.begin()+min1);
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
				//szukamy dwoch grup o min odleglosci
				if(jaccardDists[i][j]<=minJD) {
					//jesli jest jakas o mniejszej odleglosci
					if(jaccardDists[i][j]<minJD) {
						minJD = jaccardDists[i][j];	
						min1 = i;
						min2 = j;
					}
					//jesli jest o identycznej odleglosci jak aktualne minimum
					else {
						//jesli wsrod nowych jest grupa 1-elem to ma pierszenstwo
						if(linesClusters[i].size()==1 || linesClusters[j].size()==1) {
							minJD = jaccardDists[i][j];	
							min1 = i;
							min2 = j;
						}
						//jesli wsrod aktualnych nie ma grup 1-elem, a nowy zestaw w sumie ma mniej elementow, to ma pierwszenstwo
						else if(linesClusters[min1].size()!=1 && linesClusters[min2].size()!=1 && linesClusters[i].size()+linesClusters[j].size() < linesClusters[min1].size()+linesClusters[min2].size()) {
							minJD = jaccardDists[i][j];	
							min1 = i;
							min2 = j;
						}
					}
				}
			}
		}
	}



	std::cout<<"\n\n "<<linesClusters.size();

	for(int i=linesClusters.size()-1; i>=0; --i) {
		if(linesClusters[i].size()<2) linesClusters.erase(linesClusters.begin()+i);
	}

	std::cout<<" "<<linesClusters.size()<<" \n\n";
	

	Types::DrawableContainer c;
	std::vector<cv::Scalar> colors{cv::Scalar(0,0,255),cv::Scalar(0,128,255),cv::Scalar(0,255,255),cv::Scalar(0,255,128),cv::Scalar(255,128,0),cv::Scalar(0,0,255),cv::Scalar(255,0,127),
		cv::Scalar(255,0,255),cv::Scalar(0,102,0),cv::Scalar(204,204,0)};
	for(int i=0; i<linesClusters.size(); ++i) {
		for(int j=0; j<linesClusters[i].size(); ++j) {
			
			cv::Scalar color;
			if(i < colors.size()) {
				color = colors[i];
			} 
			else {
				color = cv::Scalar(0,0,0);
			}
			cv::Vec4i line = linesClusters[i][j];
			c.add(new Types::Line(cv::Point(line[0],line[1]), cv::Point(line[2],line[3]),color));
			
		}
	}
	out_linesDrawable.write(c);

}



} //: namespace VanishingPoints
} //: namespace Processors
