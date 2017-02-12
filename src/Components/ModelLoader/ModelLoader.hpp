/*!
 * \file
 * \brief 
 * \author Anna
 */

#ifndef MODELLOADER_HPP_
#define MODELLOADER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <opencv2/opencv.hpp>

#include "Types/Features.hpp"


namespace Processors {
namespace ModelLoader {

/*!
 * \class ModelLoader
 * \brief ModelLoader processor class.
 *
 * 
 */
class ModelLoader: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ModelLoader(const std::string & name = "ModelLoader");

	/*!
	 * Destructor
	 */
	virtual ~ModelLoader();

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

	// Output data streams
	Base::DataStreamOut<std::vector<Types::Features>> out_features;
	Base::DataStreamOut<std::vector<cv::Mat> > out_descriptors;
	Base::DataStreamOut<std::vector<std::vector<cv::Point2f> > > out_boundingRect;
	Base::DataStreamOut<std::vector<std::string> > out_names;

	// Handlers

	// Properties
	Base::Property<std::string> directory;
	Base::Property<std::string> pattern;

	
	// Handlers
	void LoadModels();
	void ReloadModels();

private:
	bool findFiles();
	std::vector<std::string> files;

	std::vector<Types::Features> features;
	std::vector<cv::Mat> descriptors;
	std::vector<std::vector<cv::Point2f> > boundingRect;

};

} //: namespace ModelLoader
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ModelLoader", Processors::ModelLoader::ModelLoader)

#endif /* MODELLOADER_HPP_ */
