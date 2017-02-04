/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef ModelSaver_HPP_
#define ModelSaver_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "Types/Features.hpp"


namespace Processors {
namespace ModelSaver {

/*!
 * \class ModelSaver
 * \brief ModelSaver processor class.
 *
 * 
 */
class ModelSaver: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ModelSaver(const std::string & name = "ModelSaver");

	/*!
	 * Destructor
	 */
	virtual ~ModelSaver();

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
	Base::DataStreamIn<Types::Features, Base::DataStreamBuffer::Newest,	Base::Synchronization::Mutex> in_features;
	Base::DataStreamIn<cv::Mat, Base::DataStreamBuffer::Newest,	Base::Synchronization::Mutex> in_descriptors;
	Base::DataStreamIn<std::vector<cv::Point2f>, Base::DataStreamBuffer::Newest, Base::Synchronization::Mutex> in_boundingRect;


	// Output data streams

	// Handlers

	// Properties
	Base::Property<std::string> path;

	
	// Handlers
	void addModel();
	void SaveFeatures();
	void clearAll();

	std::vector<Types::Features> allModels;
	std::vector<cv::Mat> allDescriptors;
	std::vector<std::vector<cv::Point2f> > allBoundingRect;

};

} //: namespace ModelSaver
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ModelSaver", Processors::ModelSaver::ModelSaver)

#endif /* ModelSaver_HPP_ */
