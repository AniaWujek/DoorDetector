/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef FINDDOORCORNERS_HPP_
#define FINDDOORCORNERS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "Types/Line.hpp"
#include "Types/DrawableContainer.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace FindDoorCorners {

/*!
 * \class FindDoorCorners
 * \brief FindDoorCorners processor class.
 *
 * 
 */
class FindDoorCorners: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	FindDoorCorners(const std::string & name = "FindDoorCorners");

	/*!
	 * Destructor
	 */
	virtual ~FindDoorCorners();

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
	Base::DataStreamIn<std::vector<cv::Vec4i> > in_lines;

	// Output data streams
	Base::DataStreamOut<std::vector<cv::Point> > out_corners;
	Base::DataStreamOut<std::vector<std::pair<int,int> > > out_linesPairs;
	Base::DataStreamOut<cv::Mat> out_img;
	Base::DataStreamOut <Types::DrawableContainer> out_cornersDrawable;

	// Handlers

	// Properties
	Base::Property<float> k_param;
	Base::Property<int> window_size;
	Base::Property<int> fast_param;

	
	// Handlers
	void findDoorCorners_processor();

};

} //: namespace FindDoorCorners
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("FindDoorCorners", Processors::FindDoorCorners::FindDoorCorners)

#endif /* FINDDOORCORNERS_HPP_ */
