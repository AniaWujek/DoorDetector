/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef VANISHINGPOINTS_HPP_
#define VANISHINGPOINTS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>
#include "Types/Line.hpp"
#include "Types/DrawableContainer.hpp"


namespace Processors {
namespace VanishingPoints {

/*!
 * \class VanishingPoints
 * \brief VanishingPoints processor class.
 *
 * 
 */
class VanishingPoints: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	VanishingPoints(const std::string & name = "VanishingPoints");

	/*!
	 * Destructor
	 */
	virtual ~VanishingPoints();

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

	// Output data streams
	Base::DataStreamOut<std::vector<std::vector<cv::Vec4i> > > out_linesVecs;
	Base::DataStreamOut<std::vector<cv::Point> > out_vanishingPoints;
	Base::DataStreamOut <Types::DrawableContainer> out_linesDrawable;

	// Handlers

	// Properties
	Base::Property<int> vanishing_points;
	Base::Property<int> consensus_threshold;
	Base::Property<int> hypotheses;

	
	// Handlers
	void VanishingPointsProcessor();

};

} //: namespace VanishingPoints
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("VanishingPoints", Processors::VanishingPoints::VanishingPoints)

#endif /* VANISHINGPOINTS_HPP_ */
