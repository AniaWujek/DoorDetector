/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef BOUNDINGRECT_HPP_
#define BOUNDINGRECT_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace BoundingRect {

/*!
 * \class BoundingRect
 * \brief BoundingRect processor class.
 *
 * 
 */
class BoundingRect: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	BoundingRect(const std::string & name = "BoundingRect");

	/*!
	 * Destructor
	 */
	virtual ~BoundingRect();

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
	Base::DataStreamOut<std::vector<cv::Point2f>> out_rect;

	// Handlers

	// Properties

	
	// Handlers
	void BoundingRect_processor();

};

} //: namespace BoundingRect
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("BoundingRect", Processors::BoundingRect::BoundingRect)

#endif /* BOUNDINGRECT_HPP_ */
