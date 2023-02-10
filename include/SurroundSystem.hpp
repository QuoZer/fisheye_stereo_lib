#pragma once
#include "FisheyeDewarper.hpp"
#include "StereoPair.hpp"


/**
 * The master class controlling all the cameras and stereopairs. 
 * 
 * 1. Form 'CameraModel' objects for your cameras and add them to the Surround System
 * using the 'addNewCam' method. 
 * 2. Define stereopairs by choosing the optical axis and either giving the 'CameraModel' objects 
 * or camera indexes from the previous step to the 'createStereopair' method.
 * 3. Use the 'getImage' method with the desired stereopair index to get a pair of undistorted images
 * 
 * TODO: Fix when the systems starts to return depth instead
 */
class SurroundSystem {
public:

	/// <summary>
	/// Default camera models
	/// </summary>
	enum CameraModels
	{
		PINHOLE,
		ATAN,
		SCARAMUZZA,
		MEI,
		KBModel
	};
	
	/// <summary>
	/// Image type to return. TODO: make it actually do something
	/// </summary>
	enum ImageType
	{
		RAW,
		RECTIFIED,
		DISPARITY,
		DEPTH
	};


private: //* Containers *//
	/// <summary>
	/// Holds the calibrated dewarpers
	/// </summary>
	std::vector<std::shared_ptr<FisheyeDewarper>> dewarpers; 
	/// <summary>
	/// Assigned 'CameraModel' objects
	/// </summary>
	std::vector<std::shared_ptr<CameraModel>> cameras;		// replace with a dictionary?
	/// <summary>
	/// Configured stereopairs vector
	/// </summary>
	std::vector<std::shared_ptr<Stereopair>> stereopairs;

public:
	/// <summary>
	/// Takes the cameras into the SurroundSystem
	/// </summary>
	/// <param name="readyModel">A configured camera model object ('CameraModel' inherited)</param>
	/// <returns>Index of the added camera</returns>
	int addNewCam(CameraModel& readyModel);

	//CameraModel SurroundSystem::getCameraModel(CameraModels);
	
	/// <summary>
	/// Form a stereopair out of two cameras. One camera can be a part of multiple stereopairs
	/// </summary>
	/// <param name="leftModel, rightModel"> Configured left and right cameras</param>
	/// <param name="reconstructedRes"> Desired size of the output images </param>
	/// <param name="direction"> Desired direction of stereo optical axis (TODO: implement)</param>
	/// <param name="Method"> Stereo matching method</param>
	/// <returns>Index of the created stereopair</returns>
	int createStereopair(CameraModel& leftModel, CameraModel& rightModel, cv::Size reconstructedRes, cv::Vec3d direction, StereoMethod);
	/// <summary>
	/// Form a stereopair out of two cameras. One camera can be a part of multiple stereopairs
	/// </summary>
	/// <param name="lCamIndex, rCamIndex"> Index of the configured left and right cameras </param>
	/// <param name="reconstructedRes"> Desired size of the output images </param>
	/// <param name="direction"> Desired direction of stereo optical axis (TODO: implement) </param>
	/// <param name="Method"> Stereo matching method</param>
	/// <returns>Index of the created stereopair</returns>
	int createStereopair(int lCamIndex, int rCamIndex, cv::Size reconstructedRes, cv::Vec3d direction, StereoMethod);
	
	/// <summary>
	/// Computes look-up-tables for all the declared stereopairs
	/// </summary>
	/// <param name="saveResults"> Save computed stereopairs in a file for future use (TODO: implement) </param>
	void prepareLUTs(bool saveResults);

	/// <summary>
	/// Get a number of declared stereopairs
	/// </summary>
	int getNumOfSP();

	/// <summary>
	/// Undistort input images according to the specified stereopair parameters 
	/// </summary>
	/// <param name="stereopairIndex"> Index of the chosen stereopair </param>
	/// <param name="IT"> Enum of the desired output (TODO) </param>
	/// <param name="l"> Left input image </param>
	/// <param name="r"> Right input image </param>
	/// <param name="dst"> Output image (should be of 2w*h size rn) </param>
	void getImage(int stereopairIndex, ImageType IT, cv::Mat& l, cv::Mat& r, cv::Mat& dst);

};