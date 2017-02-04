/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef QUADMODEL_HPP_
#define QUADMODEL_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "Types/Objects3D/Object3D.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace QuadModel {

/*!
 * \class QuadModel
 * \brief QuadModel processor class.
 *
 * 
 */
class QuadModel: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	QuadModel(const std::string & name = "QuadModel");

	/*!
	 * Destructor
	 */
	virtual ~QuadModel();

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
	Base::DataStreamIn<std::vector<cv::Point2f> > in_points;

	// Output data streams
	Base::DataStreamOut<Types::Objects3D::Object3D> out_model;

	// Handlers

	// Properties
	Base::Property<float> width;
	Base::Property<float> height;
	Base::Property<int> type;

	
	// Handlers
	void QuadModel_processor();

	boost::shared_ptr<Types::Objects3D::Object3D> model;
	void sizeCallback(float old_value, float new_value);
	void initModel();

};

} //: namespace QuadModel
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("QuadModel", Processors::QuadModel::QuadModel)

#endif /* QUADMODEL_HPP_ */
