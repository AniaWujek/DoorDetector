/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef SHARPENMASK_HPP_
#define SHARPENMASK_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace SharpenMask {

/*!
 * \class SharpenMask
 * \brief SharpenMask processor class.
 *
 * 
 */
class SharpenMask: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SharpenMask(const std::string & name = "SharpenMask");

	/*!
	 * Destructor
	 */
	virtual ~SharpenMask();

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
	Base::DataStreamIn<cv::Mat> in_img;

	// Output data streams
	Base::DataStreamOut<cv::Mat> out_img;

	// Handlers

	// Properties
	Base::Property<int> kernel_width;
	Base::Property<int> kernel_height;
	Base::Property<double> sigmax;
	Base::Property<double> sigmay;
	Base::Property<int> threshold;
	Base::Property<int> amount;

	
	// Handlers
	void onNewImage();

};

} //: namespace SharpenMask
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SharpenMask", Processors::SharpenMask::SharpenMask)

#endif /* SHARPENMASK_HPP_ */
