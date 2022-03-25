#pragma once
#define _USE_MATH_DEFINES
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include <numbers>
#include <math.h>
#include "Intrinsics.hpp"


/* TODO: 
*  setResolution 
*/

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

public:	///* Data *///
	double errorsum;

public:	///* Internal functions *///
	/* Tools */
	// converting corner coordinates to the center ones
	void toCenter(cv::Point& cornerPixel, cv::Size imagesize);
	// converting center coordinates to the corner ones
	void toCorner(cv::Point& centerPixel, cv::Size imagesize);

public:		///* Settings *///
	CameraModel();
	void setCamParams(cv::Size origImageSize);
	void setExtrinsics(cv::Vec3d pos, cv::Vec4d rot);

	void setIntrinsics(DistortionParams params);
	/* Projection functions */

	/*
	\brief Takes a 3D point in camera coordinates and projects it into fisheye image coordinates
	
	*/
	virtual cv::Point2d projectWorldToPixel(cv::Mat worldPoint) = NULL; 		// idk it doesnt want to return null
	/*
	\brief Takes a point in image (central coordinates) and projects it into camera coordinates

	*/
	virtual cv::Mat projectPixelToWorld(cv::Point pixel) = NULL;

};