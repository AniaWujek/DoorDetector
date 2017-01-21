/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef LINESCORNERSFITTING_HPP_
#define LINESCORNERSFITTING_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace LinesCornersFitting {

/*!
 * \class LinesCornersFitting
 * \brief LinesCornersFitting processor class.
 *
 * 
 */
class LinesCornersFitting: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	LinesCornersFitting(const std::string & name = "LinesCornersFitting");

	/*!
	 * Destructor
	 */
	virtual ~LinesCornersFitting();

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
	Base::DataStreamIn<std::vector<cv::Vec4i>> in_lines;
	Base::DataStreamIn<std::vector<cv::Point>> in_corners;
	Base::DataStreamIn<std::vector<std::pair<int,int> > > in_linesPairs;
	Base::DataStreamIn<cv::Mat> in_img;

	// Output data streams
	Base::DataStreamOut<std::vector<std::vector<cv::Point> >> out_roiVec;
	Base::DataStreamOut<cv::Mat> out_img;

	// Handlers

	// Properties
	Base::Property<int> prop;
	Base::Property<int> quality;

	
	// Handlers
	void LinesCornersFitting_processor();

};

} //: namespace LinesCornersFitting
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("LinesCornersFitting", Processors::LinesCornersFitting::LinesCornersFitting)

#endif /* LINESCORNERSFITTING_HPP_ */
