/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef HISTOGRAMEQUALIZATION_HPP_
#define HISTOGRAMEQUALIZATION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


namespace Processors {
namespace HistogramEqualization {

/*!
 * \class HistogramEqualization
 * \brief HistogramEqualization processor class.
 *
 * 
 */
class HistogramEqualization: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	HistogramEqualization(const std::string & name = "HistogramEqualization");

	/*!
	 * Destructor
	 */
	virtual ~HistogramEqualization();

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

	
	// Handlers
	void onNewImage();

};

} //: namespace HistogramEqualization
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("HistogramEqualization", Processors::HistogramEqualization::HistogramEqualization)

#endif /* HISTOGRAMEQUALIZATION_HPP_ */
