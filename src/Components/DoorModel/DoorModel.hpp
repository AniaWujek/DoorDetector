/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef DOORMODEL_HPP_
#define DOORMODEL_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include "Types/Objects3D/Object3D.hpp"

#include <opencv2/opencv.hpp>


namespace Processors {
namespace DoorModel {

/*!
 * \class DoorModel
 * \brief DoorModel processor class.
 *
 * 
 */
class DoorModel: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	DoorModel(const std::string & name = "DoorModel");

	/*!
	 * Destructor
	 */
	virtual ~DoorModel();

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
	Base::DataStreamOut<Types::Objects3D::Object3D> out_doorModel;

	// Handlers

	// Properties
	Base::Property<float> width;
	Base::Property<float> height;

	
	// Handlers
	void DoorModel_processor();

	boost::shared_ptr<Types::Objects3D::Object3D> door_model;
	void sizeCallback(float old_value, float new_value);
	void initModel();

};

} //: namespace DoorModel
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("DoorModel", Processors::DoorModel::DoorModel)

#endif /* DOORMODEL_HPP_ */
