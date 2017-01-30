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

namespace Processors {
namespace BehaviourSwitch {

BehaviourSwitch::BehaviourSwitch(const std::string & name) :
		Base::Component(name)  {

}

BehaviourSwitch::~BehaviourSwitch() {
}

void BehaviourSwitch::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_behaviour", &out_behaviour);
	// Register handlers
	registerHandler("BehaviourSwitch_processor", boost::bind(&BehaviourSwitch::BehaviourSwitch_processor, this));

}

bool BehaviourSwitch::onInit() {

	return true;
}

bool BehaviourSwitch::onFinish() {
	return true;
}

bool BehaviourSwitch::onStop() {
	return true;
}

bool BehaviourSwitch::onStart() {
	return true;
}

void BehaviourSwitch::BehaviourSwitch_processor() {
}



} //: namespace BehaviourSwitch
} //: namespace Processors
