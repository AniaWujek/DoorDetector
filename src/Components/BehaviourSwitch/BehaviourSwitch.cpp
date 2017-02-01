/*!
 * \file
 * \brief
 * \author Anna
 */

#include <memory>
#include <string>

#include "BehaviourSwitch.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <iostream>

namespace Processors {
namespace BehaviourSwitch {

BehaviourSwitch::BehaviourSwitch(const std::string & name) :
		Base::Component(name) , 
		ros_topic_name("ros_topic_name", std::string("mytopic")), 
		ros_namespace("ros_namespace", std::string("mynamespace")) {
	registerProperty(ros_topic_name);
	registerProperty(ros_namespace);

}

BehaviourSwitch::~BehaviourSwitch() {
}

void BehaviourSwitch::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_behaviour", &out_behaviour);
	registerStream("out_imgPath", &out_imgPath);
	registerStream("out_modelsPath", &out_modelsPath);
	// Register handlers
	registerHandler("BehaviourSwitch_processor", boost::bind(&BehaviourSwitch::BehaviourSwitch_processor, this));
	addDependency("BehaviourSwitch_processor", NULL);

}

bool BehaviourSwitch::onInit() {
	static char * tmp = NULL;
	static int tmpi;
	ros::init(tmpi, &tmp, std::string(ros_namespace), ros::init_options::NoSigintHandler);
	nodeHandle = new ros::NodeHandle;
	subscriber = nodeHandle->subscribe(ros_topic_name, 1, &BehaviourSwitch::callback, this);

	behaviour = "x";
	imgPath = "x";
	modelsPath = "x";


	return true;
}

bool BehaviourSwitch::onFinish() {
	delete nodeHandle;
	return true;
}

bool BehaviourSwitch::onStop() {
	return true;
}

bool BehaviourSwitch::onStart() {
	return true;
}

void BehaviourSwitch::callback(const door_detector::Order_simple::ConstPtr& order) {
	if(!own_spin) return;
	std::string b, i, m;
	b = order->behaviour;
	i = order->img_path;
	m = order->models_path;	
	if(b != behaviour) behaviour = b;
	if(i != imgPath) imgPath = i;
	if(m != modelsPath) modelsPath = m;

	int b_code = 0;
	if(behaviour == "door_localization") b_code = 1;
	if(behaviour == "elements_localization") b_code = 2;

	out_behaviour.write(b_code);
	out_imgPath.write(imgPath);
	out_modelsPath.write(modelsPath);


	CLOG(LNOTICE) << "\nReceived: " << *order;
}

void BehaviourSwitch::BehaviourSwitch_processor() {
	own_spin = true;
	ros::spinOnce();
	own_spin = false;

}



} //: namespace BehaviourSwitch
} //: namespace Processors
