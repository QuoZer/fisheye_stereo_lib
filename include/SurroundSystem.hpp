#pragma once
#include "FisheyeDewarper.hpp"
#include "Stereopair.hpp"

/*
 - User adds cameras with their relative positions
 - User picks camera pairs defining  a stereopair
	* User sets 3D optical axis ?
	* 3D optical axis defined by positions?
 -   

*/


class SurroundSystem {
public:
	//*   Model constants   *//
	enum CameraModels
	{
		PINHOLE,
		ATAN,
		SCARAMUZZA,
		MEI,
		KBModel
	};

	enum ImageType
	{
		RAW,
		RECTIFIED,
		DISPARITY,
		DEPTH
	};


private: //* Containers *//
	std::vector<std::shared_ptr<FisheyeDewarper>> dewarpers; 
	std::vector<std::shared_ptr<CameraModel>> cameras;		// replace with a dictionary?
	std::vector<std::shared_ptr<Stereopair>> stereopairs;

public:
	int addNewCam(CameraModel& readyModel);

	//CameraModel SurroundSystem::getCameraModel(CameraModels);
	int createStereopair(const CameraModel& leftModel, const CameraModel& rightModel, cv::Size reconstructedRes, cv::Vec3d direction, StereoMethod);
	int createStereopair(int lCamIndex, int rCamIndex, cv::Size reconstructedRes, cv::Vec3d direction, StereoMethod);
	
	int loadLUTs();
	void prepareLUTs(bool saveResults);

	int getNumOfSP();
	void getImage(int stereopairIndex, ImageType IT, cv::Mat& l, cv::Mat& r, cv::Mat& dst);

};