#define _USE_MATH_DEFINES
#pragma once
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <math.h>
#include "models.h"

/// <summary>
/// Camera model agnostic class for removing distortions in RoI. Usually operated by the SurroundSystem class
/// </summary>
class FisheyeDewarper
{
private:	///* Parameters *///
	const double PI = 3.141592653589; // M_PI;
	float xFov;							// output image fov
	float yFov;							// or 16:9 equivalent
	cv::Size oldSize;					// input image size
	cv::Size newSize;					// output image size
	/* RPY angles for the world point rotator*/
	float yaw;
	float pitch;
	float roll;
	/* Camera models to be used for dewarping (and setting intrinsics) */
	
	/// <summary>
	/// Pointer to a camera model described by inheritors of the "CameraModel" class. Allows to use any camera model
	/// </summary>
	std::shared_ptr<CameraModel> cameraModel;		
	std::shared_ptr<PinholeModel> pinhole;

private:	///* Data *///
	double errorsum;
	/* Intrinsics */
	std::vector <double> scara_polynom;		// Scaramuzza model coefficients		// DEPRICATED
	std::vector <double> mei_polynom;		// Mei model coefficients			// DEPRICATED
	cv::Vec2d   centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Stretch matrix
	double lambda;						// Scale factor 
	/* Structures */
	cv::Mat map1;						// x map
	cv::Mat map2;						// y map
	std::vector<cv::Point> frameBorder; // Set of border points to draw on the original image 

private:	///* Internal functions *///

	/// <summary>
	/// create empty maps or clear old ones
	/// </summary>
	void createMaps();

	/* Transformations */

	/// <summary>
	/// Transforms a pixel from opencv corner reference frame to the central one
	/// </summary>
	/// <param name="cornerPixel"> Pixel that needs to be transformed </param>
	/// <param name="imagesize"> Image size </param>
	void toCenter(cv::Point& cornerPixel, cv::Size imagesize);

	/// <summary>
	/// Transforms a pixel from central reference frame to the opencv one
	/// </summary>
	/// <param name="centerPixel"> Pixel that needs to be transformed </param>
	/// <param name="imagesize"> Image size </param>
	void toCorner(cv::Point& centerPixel, cv::Size imagesize);

	/// <summary>
	/// Rotates the 3D point in camera frame according to the stereoapir parameters
	/// </summary>
	/// <param name="worldPoint"> 3D point that needs to be rotated </param>
	/// <returns> Rotated 3D point </returns>
	cv::Mat rotatePoint(cv::Mat worldPoint);

	/* Projection functions */
	cv::Point reverseScaramuzza(cv::Point pixel);	// DEPRICATED
	/* Tools */
	void setFovWide(float wFov);

public:		///* Settings *///
	FisheyeDewarper();
	FisheyeDewarper(std::shared_ptr<CameraModel> model);

	/// <summary>
	/// Sets the image parameters based on the size and fov width info. Assumes rectangular pixels
	/// </summary>
	/// <param name="oldsize"> Size of the raw camera image </param>
	/// <param name="newsize"> Size of the output images </param>
	/// <param name="wideFov"> x-axis FoV in degrees </param>
	void setSize(cv::Size oldsize, cv::Size newsize, float wideFov);

	/// <summary>
	/// Set the rotation angles from original fisheye optical axis direction to the desired one. 
	/// Yaw, pitch, roll convention, values in degrees.
	/// </summary>
	/// <param name="yaw"></param>
	/// <param name="pitch"></param>
	/// <param name="roll"></param>
	void setRpy(float yaw, float pitch, float roll);

	/// <summary>
	/// Set the rotation angles from original fisheye optical axis direction to the desired one. 
	/// Yaw, pitch, roll convention, values in degrees.
	/// </summary>
	/// <param name="angles"> Vector of angle values in order according to the convention </param>
	void setRpy(cv::Vec3d angles);

	/// <summary>
	/// Set the rotation angles from original fisheye optical axis direction to the desired one. 
	/// Yaw, pitch, roll convention, values in radians.
	/// </summary>
	/// <param name="rad_angles">  Vector of angle values in order according to the convention </param>
	void setRpyRad(cv::Vec3d rad_angles);

	/// <summary>
	/// Return the outline of undistorted area as a vector if points
	/// </summary>
	/// <returns></returns>
	std::vector<cv::Point> getBorder();

public:		/*  */
	/// <summary>
	/// Compute look-up table for the camera
	/// </summary>
	void fillMaps();

	/// <summary>
	/// Return an undistorted image 
	/// </summary>
	/// <param name="inputImage"> Fisheye input image </param>
	/// <returns> Undistorted image </returns>
	cv::Mat dewarpImage(cv::Mat inputImage);

	//TODO:
	void saveMaps(std::string prefix);
	int loadMaps(std::string prefix);
	
};


