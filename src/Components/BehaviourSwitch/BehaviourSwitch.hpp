/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef BEHAVIOURSWITCH_HPP_
#define BEHAVIOURSWITCH_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace BehaviourSwitch {

/*!
 * \class BehaviourSwitch
 * \brief BehaviourSwitch processor class.
 *
 * 
 */
class BehaviourSwitch: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	BehaviourSwitch(const std::string & name = "BehaviourSwitch");

	/*!
	 * Destructor
	 */
	virtual ~BehaviourSwitch();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	// Input data streams

	// Output data streams
	Base::DataStreamOut<int> out_behaviour;

	// Handlers

	// Properties

	
	// Handlers
	void BehaviourSwitch_processor();

};

} //: namespace BehaviourSwitch
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("BehaviourSwitch", Processors::BehaviourSwitch::BehaviourSwitch)

#endif /* BEHAVIOURSWITCH_HPP_ */
