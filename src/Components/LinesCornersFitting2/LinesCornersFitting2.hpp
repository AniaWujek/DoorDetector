/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef LinesCornersFitting2_HPP_
#define LinesCornersFitting2_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace LinesCornersFitting2 {

/*!
 * \class LinesCornersFitting2
 * \brief LinesCornersFitting2 processor class.
 *
 * 
 */
class LinesCornersFitting2: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	LinesCornersFitting2(const std::string & name = "LinesCornersFitting2");

	/*!
	 * Destructor
	 */
	virtual ~LinesCornersFitting2();

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
	Base::DataStreamIn<std::vector<cv::Point2f>> in_corners;
	Base::DataStreamIn<std::vector<std::pair<int,int> > > in_linesPairs;
	Base::DataStreamIn<cv::Mat> in_img;

	// Output data streams
	Base::DataStreamOut<std::vector<cv::Point2f> > out_door;
	Base::DataStreamOut<cv::Mat> out_img;

	// Handlers

	// Properties
	Base::Property<int> prop;
	Base::Property<int> quality;
	Base::Property<bool> dev_choice;
	Base::Property<int> hsv_channel;

	
	// Handlers
	void LinesCornersFitting2_processor();

};


} //: namespace LinesCornersFitting2
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("LinesCornersFitting2", Processors::LinesCornersFitting2::LinesCornersFitting2)

#endif /* LinesCornersFitting2_HPP_ */
