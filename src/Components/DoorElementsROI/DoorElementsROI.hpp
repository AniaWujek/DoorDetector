/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef DOORELEMENTSROI_HPP_
#define DOORELEMENTSROI_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace DoorElementsROI {

/*!
 * \class DoorElementsROI
 * \brief DoorElementsROI processor class.
 *
 * 
 */
class DoorElementsROI: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	DoorElementsROI(const std::string & name = "DoorElementsROI");

	/*!
	 * Destructor
	 */
	virtual ~DoorElementsROI();

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
	Base::DataStreamIn<std::vector<cv::Point>> in_door;
	Base::DataStreamIn<cv::Mat> in_img;

	// Output data streams
	Base::DataStreamOut<std::vector<std::vector<cv::Point> > > out_elementsROI;
	Base::DataStreamOut<cv::Mat> out_img;

	// Handlers

	// Properties
	Base::Property<float> ratio;
	Base::Property<int> contour_n;
	Base::Property<int> epsilon;
	Base::Property<int> horizontal_thresh;

	
	// Handlers
	void DoorElementsROI_processor();

};

} //: namespace DoorElementsROI
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("DoorElementsROI", Processors::DoorElementsROI::DoorElementsROI)

#endif /* DOORELEMENTSROI_HPP_ */
