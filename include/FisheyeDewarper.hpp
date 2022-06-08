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

//#define SCARAMUZZA		10
//#define ATAN			20
//#define REV_SCARAMUZZA	30

/* TODO: -Constructor, 
		 -
*/

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
	/* A camera model to be used for dewarping (and setting intrinsics) */
	std::shared_ptr<CameraModel> cameraModel;
	std::shared_ptr<PinholeModel> pinhole;

private:	///* Data *///
	double errorsum;
	/* Intrinsics */
	std::vector <double> scara_polynom;		// Scaramuzza model coefficients
	std::vector <double> mei_polynom;		// Mei model coefficients
	cv::Vec2d   centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Stretch matrix
	double lambda;						// Scale factor 
	/* Structures */
	cv::Mat map1;						// x map
	cv::Mat map2;						// y map
	std::vector<cv::Point> frameBorder; //border to draw on the original image 

private:	///* Internal functions *///
	void createMaps();
	/* Transformations */
	void toCenter(cv::Point& cornerPixel, cv::Size imagesize);
	void toCorner(cv::Point& centerPixel, cv::Size imagesize);
	cv::Mat rotatePoint(cv::Mat worldPoint);
	/* Projection functions */
	cv::Point reverseScaramuzza(cv::Point pixel);	// DEPRICATED
	/* Tools */
	void setFovWide(float wFov);

public:		///* Settings *///
	FisheyeDewarper();
	FisheyeDewarper(std::shared_ptr<CameraModel> model);
	void setSize(cv::Size oldsize, cv::Size newsize, float wideFov);
	void setRpy(float yaw, float pitch, float roll);
	void setRpy(cv::Vec3d angles);
	void setRpyRad(cv::Vec3d rad_angles);

	std::vector<cv::Point> getBorder();

public:		/*  */
	void fillMaps();
	cv::Mat dewarpImage(cv::Mat inputImage);
	void saveMaps(std::string prefix);
	int loadMaps(std::string prefix);
	
};


