/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef IMPROVELINES_HPP_
#define IMPROVELINES_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "Types/Line.hpp"
#include "Types/DrawableContainer.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace ImproveLines {

/*!
 * \class ImproveLines
 * \brief ImproveLines processor class.
 *
 * 
 */
class ImproveLines: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ImproveLines(const std::string & name = "ImproveLines");

	/*!
	 * Destructor
	 */
	virtual ~ImproveLines();

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
	Base::DataStreamIn<std::vector<cv::Vec4i> > in_lines;
	//Base::DataStreamIn<cv::Mat> in_img;

	// Output data streams
	Base::DataStreamOut<std::vector<cv::Vec4i> > out_lines;
	Base::DataStreamOut <Types::DrawableContainer> out_linesDrawable;

	// Handlers

	// Properties
	Base::Property<float> collinearRatio;
	Base::Property<float> shortRatio;
	Base::Property<float> closeRatio;
	Base::Property<int> width;
	Base::Property<int> height;
	
	// Handlers
	void improveLinesProcessor();

};

} //: namespace ImproveLines
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ImproveLines", Processors::ImproveLines::ImproveLines)

#endif /* IMPROVELINES_HPP_ */
