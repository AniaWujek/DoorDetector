/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef MODELFAKER_HPP_
#define MODELFAKER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace ModelFaker {

/*!
 * \class ModelFaker
 * \brief ModelFaker processor class.
 *
 * 
 */
class ModelFaker: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ModelFaker(const std::string & name = "ModelFaker");

	/*!
	 * Destructor
	 */
	virtual ~ModelFaker();

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
	Base::DataStreamOut<cv::Mat> out_img;

	// Handlers

	// Properties
	Base::Property<int> positionX;
	Base::Property<int> positionY;
	Base::Property<int> width;
	Base::Property<int> height;
	Base::Property<int> radius;

	
	// Handlers
	void ModelFaker_processor();

};

} //: namespace ModelFaker
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ModelFaker", Processors::ModelFaker::ModelFaker)

#endif /* MODELFAKER_HPP_ */
