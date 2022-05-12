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
	void setIntrinsics(ScaraParams sp);
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

class KBModel : public CameraModel
{
private:
	double k2, k3, k4, k5;

public:
	KBModel();
	KBModel(KBParams mp);

	void setIntrinsics(KBParams mp);

	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel);
	void KBModel::backprojectSymmetric(cv::Point pxl, double& theta, double& phi);
};


class MeiModel : public CameraModel
{
private:
	double eps;
	std::vector<double> mei_polynom;

	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);


};