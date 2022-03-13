#pragma once
#include "FisheyeDewarper.hpp"


enum StereoMethod
{
	BM,
	SGBM
};

class Stereopair
{
	cv::StereoMatcher* matcher;

	std::shared_ptr<FisheyeDewarper> leftDewarper;
	std::shared_ptr<FisheyeDewarper> rightDewarper;

	std::shared_ptr<CameraModel> leftCamera;
	std::shared_ptr<CameraModel> rightCamera; // replace with positions? 
public:
	cv::Size outputSize;

private:
	// get the transaltion and rotation from cam2 to cam1
	void calcExtrinsics(cv::OutputArray mtx, cv::InputArray poseCam1, cv::InputArray poseCam2);

public:
	Stereopair(std::shared_ptr<CameraModel> lCam, std::shared_ptr<FisheyeDewarper> lDWarp,
		std::shared_ptr<CameraModel> rCam, std::shared_ptr<FisheyeDewarper> rDWarp, cv::Size outSize);
	Stereopair(std::shared_ptr<CameraModel> lCam, std::shared_ptr<FisheyeDewarper> lDWarp,
		std::shared_ptr<CameraModel> rCam, std::shared_ptr<FisheyeDewarper> rDWarp, cv::Size outSize, StereoMethod sm);

	void fillMaps();
	// 
	int getRemapped(cv::Mat& left, cv::Mat& right, cv::Mat& leftRemapped, cv::Mat& rightRemapped);
	int getDisparity(cv::OutputArray& dist, cv::InputArray& leftImage, cv::InputArray& rightImage);
	int getDepth(cv::OutputArray& dist, cv::InputArray& leftImage, cv::InputArray& rightImage);
	void setStereoMethod(StereoMethod sm);
	// calculate and set the best roll,pitch,yaw values fir each camera in stereopir 
	void setOptimalDirecton();
	void setDirection();

};