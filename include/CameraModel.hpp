#pragma once
#define _USE_MATH_DEFINES
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include <math.h>
#include "Intrinsics.hpp"


/* TODO: 
*  setResolution 
*/

/// <summary>
/// A parent class for all the camera models. Inherit overriding virtual functions 'projectWorldToPixel' and 'projectPixelToWorld'
/// to add your own camera models
/// </summary>
class CameraModel
{
public:	///* Parameters *///
	float xFov;							
	float yFov;							
	cv::Size oldSize;					
	cv::Size newSize;					
	int modelID;
	// TODO: some kind of position storage 
	cv::Vec3d position;
	cv::Vec4d rotation;

	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;

public:	///* Data *///
	double errorsum;

public:	///* Internal functions *///
	/* Tools */


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

public:		///* Settings *///
	CameraModel();

	/// <summary>
	/// Set parameters of the fisheye camera. Only resolution matters at this point
	/// </summary>
	/// <param name="origImageSize"> Image resolution </param>
	void setCamParams(cv::Size origImageSize);

	/// <summary>
	/// Set camera position and orientation on your robot/setup. Keep in mind that all cameras should use common reference frame
	/// </summary>
	/// <param name="pos"> Position in xyz coordinates </param>
	/// <param name="rot"> Orientation in quaternion </param>
	void setExtrinsics(cv::Vec3d pos, cv::Vec4d rot);

	// trying to use one agnostic function to set parameters no matter the parameters
	void setIntrinsics(DistortionParams params);

	/* Projection functions */

	/// <summary>
	/// Virtual. Takes a 3D point in camera coordinates and projects it into fisheye image coordinates
	/// </summary>
	/// <param name="worldPoint"> 3D point in camera coordinates </param>
	/// <returns> 2D point in image plane </returns>
	virtual cv::Point2d projectWorldToPixel(cv::Mat worldPoint) = 0; 		// idk it doesnt want to return null

	/// <summary>
	///  Virtual. Takes a point in image (central coordinates) and projects it into camera coordinates. 
	/// </summary>
	/// <param name="pixel"> 2D point in image plane </param>
	/// <returns> 3D point in camera coordinates </returns>
	virtual cv::Mat projectPixelToWorld(cv::Point pixel) = 0;

};