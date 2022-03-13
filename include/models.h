#pragma once 
#include "CameraModel.h"


class PinholeModel : public CameraModel
{
	/* Parameters as in parent */

public:	///* Projection functions *///
	PinholeModel();
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel);
	void setIntrinsics(cv::Size newsize, float wideFov);
};


class ScaramuzzaModel : public CameraModel
{
private:	///* Intrinsics *///
	std::vector <double> scara_polynom;		// Scaramuzza model coefficients
	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Stretch matrix
	double lambda;						// Scale factor 

public:	///* Projection functions *///
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel);

public:
	void setIntrinsics(std::initializer_list<double>, cv::Vec2d centerOffset, 
		cv::Matx22d stretchMatrix, double scaleFactor);
};

class AtanModel : public CameraModel
{
private:	///* Intrinsics *///
	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Stretch matrix
	double lambda;						// Scale factor 

public:	///* Projection functions *///
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel) { return cv::Mat(oldSize, CV_8UC3, cv::Scalar(0, 0, 0)); }
	
public:
	void setIntrinsics(cv::Vec2d centerOffset, cv::Matx22d stretchMatrix, double scaleFactor) { return; };

};

class MeiModel : public CameraModel
{
private:
	double k2, k3, k4, k5, mu, mv;

public:
	MeiModel();
	MeiModel(double k2, double k3, double k4, double k5, double mu, double mv);

	void setIntrinsics(double k2, double k3, double k4, double k5, double mu, double mv);

	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel);
	void MeiModel::backprojectSymmetric(cv::Point pxl, double& theta, double& phi);
};