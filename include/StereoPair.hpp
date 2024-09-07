#pragma once
#include "FisheyeDewarper.hpp"

/// <summary>
/// Available stereomatching methods
/// </summary>
enum StereoMethod
{
	BM,
	SGBM
};

/// <summary>
/// Class to manage two assigned cameras and compute depth
/// </summary>
class Stereopair
{
	/// <summary>
	/// Configured stereo matcher (SGBM/ SGM)
	/// </summary>
	cv::Ptr<cv::StereoMatcher> matcher;
	
	/// <summary>
	/// Dewarpers for left and right cameras
	/// </summary>
	std::shared_ptr<FisheyeDewarper> leftDewarper;
	std::shared_ptr<FisheyeDewarper> rightDewarper;

	/// <summary>
	/// Left and right camera objects.
	/// </summary>
	std::shared_ptr<CameraModel> leftCamera;
	std::shared_ptr<CameraModel> rightCamera;  
public:
	/// <summary>
	/// The size of output images
	/// </summary>
	cv::Size outputSize;

private:
	// TODO: get the transaltion and rotation from cam2 to cam1
	void calcExtrinsics(cv::OutputArray mtx, cv::InputArray poseCam1, cv::InputArray poseCam2);
	// too to get sign of a number
	int sign(double x);

public:
	/// <summary>
	/// Construct stereopair object. Meant to be primerely used by the SurroundSystem object
	/// </summary>
	/// <param name="lCam"> Left configured camera</param>
	/// <param name="lDWarp"> Left dewarper </param>
	/// <param name="rCam"> Right configured camera </param>
	/// <param name="rDWarp"> Right dewarper </param>
	/// <param name="outSize"> Dewarped image size </param>
	Stereopair(std::shared_ptr<CameraModel> lCam, std::shared_ptr<FisheyeDewarper> lDWarp,
		std::shared_ptr<CameraModel> rCam, std::shared_ptr<FisheyeDewarper> rDWarp, cv::Size outSize);
	
	/// <summary>
	/// Translates quaternion values into Euler angles
	/// </summary>
	/// <param name="quaternion"></param>
	/// <returns></returns>
	cv::Vec3d quatToRpy(cv::Vec4d quaternion);

	/// <summary>
	/// Compute look-up-tables for the attached cameras
	/// </summary>
	void fillMaps();

	/// <summary>
	/// Save maps of the associated cameras
	/// </summary>
	void saveMaps(int index, std::string& systemId);

	/// <summary>
	/// Loads the maps of the associated cameras
	/// </summary>
	/// <param name="index"></param>
	/// <param name="systemId"></param>
	void loadMaps(int index, const char* systemId);

	/// <summary>
	/// Returns look-up tables for the associated cameras
	/// </summary>
	/// <returns></returns>
	std::vector<cv::Mat> getMaps();
	
	/// <summary>
	/// Get undistorted images for stereo
	/// </summary>
	/// <param name="left"> left input image </param>
	/// <param name="right"> right input image </param>
	/// <param name="leftRemapped"> left output image </param>
	/// <param name="rightRemapped"> right output image </param>
	/// <returns> nothing </returns>
	int getRemapped(cv::Mat& left, cv::Mat& right, cv::Mat& leftRemapped, cv::Mat& rightRemapped);

	/// <summary>
	/// Doesn't do anything for now. Supposed to return reverse depth (TODO: implement)
	/// </summary>
	/// <param name="dist"></param>
	/// <param name="leftImage"></param>
	/// <param name="rightImage"></param>
	/// <returns></returns>
	int getDisparity(cv::OutputArray& dist, cv::InputArray& leftImage, cv::InputArray& rightImage);

	/// <summary>
	/// Doesn't do anything for now. Supposed to return depth (TODO: implement)
	/// </summary>
	/// <param name="dist"></param>
	/// <param name="leftImage"></param>
	/// <param name="rightImage"></param>
	/// <returns></returns>
	int getDepth(cv::OutputArray& dist, cv::InputArray& leftImage, cv::InputArray& rightImage);

	/// <summary>
	/// Loads parameters for the stereomatcher from a file
	/// </summary>
	void loadStereoSGBMParameters(const std::string& filename, cv::Ptr<cv::StereoSGBM>& matcher);

	/// <summary>
	/// Creates stereomatcher object based on the chosen type (TODO: implement)
	/// </summary>
	/// <param name="sm"></param>
	/// <param name="params_path"></param>
	void setStereoMethod(StereoMethod sm, std::string params_path);

	/// <summary>
	/// Calculate and set the best roll,pitch,yaw values for each virtual camera in the stereopair 
	/// based on their direction. TODO: implement?
	/// </summary>
	void setOptimalDirecton();

	/// <summary>
	/// Manually set rpy for cameras 
	/// </summary>
	void setDirection();

};