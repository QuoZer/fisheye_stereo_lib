#pragma once
#define _USE_MATH_DEFINES
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <vector>
#include <numbers>
#include <math.h>


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
	void toCenter(cv::Point& cornerPixel, cv::Size imagesize)
	{
		cornerPixel.x = cornerPixel.x - imagesize.width / 2;
		cornerPixel.y = -cornerPixel.y + imagesize.height / 2;
	}
	// converting center coordinates to the corner ones
	void toCorner(cv::Point& centerPixel, cv::Size imagesize)
	{
		centerPixel.x = centerPixel.x + imagesize.width / 2;
		centerPixel.y = -centerPixel.y + imagesize.height / 2;
	}

public:		///* Settings *///
	//CameraModel() {}
	virtual void setCamParams(cv::Size origImageSize)
	{
		oldSize = origImageSize;    
	}
	void setExtrinsics(cv::Vec3d pos, cv::Vec4d rot)
	{
		position = pos;
		rotation = rot;
	}
	void setIntrinsics(cv::Matx22d stretchMatrix, double scaleFactor);
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